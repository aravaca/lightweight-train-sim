import math
import json
import asyncio
import time
import os

from dataclasses import dataclass
from collections import deque
from typing import Optional, List, Tuple

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

# ------------------------------------------------------------
# Config
# ------------------------------------------------------------
DEBUG = False  # 디버그 로그를 보고 싶으면 True
# I realized that the soft stop logic does not make sense and makes the simulation less realistic 
# soft_stop_di = 10.0 
# soft_stop_const = -0.18
air_brake_vi = 12.0  # km/h 이하에서 공기제동 밸브 지연 반영 시작
test_notch = 3  # TASC 릴렉스 시도 시점에 비교할 목표 노치
test_margin = 0  # TASC 릴렉스 시도 시점에 비교할 거리 마진 (m)
# ------------------------------------------------------------
# Data classes
# ------------------------------------------------------------

@dataclass
class Vehicle:
    name: str = "EMU-233-JR-East"
    mass_t: float = 39.9
    a_max: float = 1.0
    j_max: float = 0.4
    notches: int = 10
    notch_accels: list = None
    tau_cmd: float = 0.150
    tau_brk: float = 0.250
    mass_kg: float = 39900
    # Vehicle 클래스 내
    maxSpeed_kmh: float = 140.0
    forward_notches: int = 5
    forward_notch_accels: list = None  # [-1.5, -1.1] 등


    # --- 새로 추가 ---
    T_max_kN: float = 0.0 # 최대 견인력(kN) ex) KTX-1: 382
    P_max_kW: float = 0.0 # 정격/최대 출력(kW) ex) KTX-1: 13200
    type: str = "일반"  # 차량 유형 (일반, 고속(200km/h 이상) 등)

    # Davis 계수 (열차 전체) : F = A0 + B1 * v + C2 * v^2 [N], v[m/s], 기본값 (나중에 다시 열차 type에 맞게 재계산됨)
    A0: float = 1200.0
    B1: float = 30.0
    C2: float = 8.0

    # 공력/기본 파라미터
    C_rr: float = 0.005
    rho_air: float = 1.225
    Cd: float = 1.8
    A: float = 10.0

    # --- Davis 자동추정용 튜닝 파라미터(추가) ---
    davis_k0: float = 0.0017        # A0 = k0 * m * g
    davis_m_ref: float = 200000.0   # B1 기준 질량(kg) = 200t
    davis_B1_ref: float = 40.0      # B1 기준값 (N·s/m) @ 200t

    def calibrate_C2_from_power(self, v_target_kmh: float = 300.0, eta: float = 0.85):
        if self.P_max_kW <= 0:
            return
        v = v_target_kmh / 3.6
        P = self.P_max_kW * 1000.0 * eta
        F_req = P / max(0.1, v) # 목표 속도에서 필요한 저항력
        self.C2 = max(0.0, (F_req - self.A0 - self.B1 * v) / (v * v))

    # new version of recompute_davis with type handling
    def recompute_davis(self, mass_kg: Optional[float] = None):
        """현재 총질량(kg)과 차량 Type에 맞춰 A0, B1, C2를 현실적으로 재계산"""
        m = float(mass_kg) if mass_kg is not None else float(self.mass_kg)
        
        # 1. [핵심] 차량 타입에 따른 물리 상수 설정 (분기 처리)
        if self.type == "고속":
            current_Cd = 0.22 # 0.15~0.30 권장 시작점
            current_A = 10.5 # 9~12 m^2 정도에서 튜닝
            tech_efficiency = 0.85
        else:
            current_Cd = 1.1 # 0.8~1.4
            current_A = 9.5
            tech_efficiency = 1.0

        # 2. 클래스 내부 변수 업데이트 (나중에 확인용)
        self.Cd = current_Cd
        self.A = current_A

        # 3. Davis 계수 계산
        
        # [C2] 공기 항력 (Aerodynamic Drag)
        # 고속일수록 이 값이 지배적입니다. (v^2 비례)
        # 고속열차는 Cd가 낮아서 질량이 무거워도 C2가 지하철보다 훨씬 작게 나옵니다.
        self.C2 = 0.5 * self.rho_air * self.Cd * self.A

        # [A0] 주행 저항 (Rolling Resistance)
        # A0 = k0 * m * g (tech_efficiency 적용)
        # 고속열차는 베어링 성능이 좋아 마찰이 조금 덜하다고 가정
        self.A0 = (self.davis_k0 * tech_efficiency) * m * 9.81

        # [B1] 속도 비례 저항 (Mechanical/Flange)
        # B1 = Ref * (m / m_ref)
        # 고속열차는 주행 안정 장치(Yaw Damper) 덕분에 뱀동(Hunting) 저항이 적음 -> 0.8배 보정
        b1_factor = 0.8 if self.type == "고속" else 1.0
        self.B1 = (self.davis_B1_ref * b1_factor) * (m / max(1.0, self.davis_m_ref))
        
        if self.type =="고속":
            m_ref = 383000 #TGV Reseau Data
            k = m / m_ref
            self.A0 = 2700.0 *k     
            self.B1 = 118.8*k
            self.C2 = 6.6096
            return

        if DEBUG:
            print(f"[Davis Recompute] Type={self.type}, Mass={m/1000:.1f}t")
            print(f"   -> Result: A0={self.A0:.1f}, B1={self.B1:.2f}, C2={self.C2:.2f}")
            print(f"   -> Params: Cd={self.Cd}, A={self.A}, TechFactor={tech_efficiency}")

    def update_mass(self, length: int):
        """편성 량 수에 맞춰 총 질량(kg)을 업데이트"""
        self.mass_kg = self.mass_t * 1000 * length
        # ★ 총질량 반영 후 Davis 재계산
        self.recompute_davis(self.mass_kg)

    @classmethod
    def from_json(cls, filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        mass_t = data.get("mass_t", 200.0)
        obj = cls(
            name=data.get("name", "EMU-233-JR-East"),
            a_max=data.get("a_max", 1.0),
            j_max=data.get("j_max", 0.8),
            notches=data.get("notches", 8),
            notch_accels=data.get(
                "notch_accels",
                [-1.5, -1.10, -0.95, -0.80, -0.65, -0.50, -0.35, -0.20, 0.0],
            ),
            maxSpeed_kmh=data.get("maxSpeed_kmh", 140.0),
            forward_notches=data.get("forward_notches", 5),
            forward_notch_accels=data.get("forward_notch_accels", [ 0.250, 0.287, 0.378, 0.515, 0.694 ]),
            tau_cmd=data.get("tau_cmd_ms", 150) / 1000.0,
            tau_brk=data.get("tau_brk_ms", 250) / 1000.0,
            mass_t=mass_t,
            mass_kg=mass_t * 1000,

            # 새 필드
            T_max_kN=data.get("T_max_kN", 0.0),
            P_max_kW=data.get("P_max_kW", 0.0),
            type=data.get("type", "일반"),

            # 초기값(로드 시점 값; 재계산으로 덮어씀)
            A0=data.get("davis_A0", 1200.0),
            B1=data.get("davis_B1", 30.0),
            C2=data.get("davis_C2", 8.0),

            C_rr=0.005,
            rho_air=data.get("rho_air", 1.225),
            Cd=data.get("Cd", 1.8),
            A=data.get("A", 10.0),

            # Davis 추정용 옵션 로드(없으면 기본)
            davis_k0=data.get("davis_k0", 0.0017),
            davis_m_ref=data.get("davis_m_ref", 200000.0),
            davis_B1_ref=data.get("davis_B1_ref", 40.0),
        )
        # ★ 총질량 기준으로 Davis 재계산(최초 1회)
        obj.recompute_davis(obj.mass_kg)
        return obj


@dataclass
class Scenario:
    L: float = 500.0
    v0: float = 25.0
    grade_percent: float = 0.0
    mu: float = 1.0
    dt: float = 0.005 #0.01

    @classmethod
    def from_json(cls, filepath):
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        v0_kmph = data.get("v0", 25.0)
        v0_ms = v0_kmph / 3.6
        return cls(
            L=data.get("L", 500.0),
            v0=v0_ms,
            grade_percent=data.get("grade_percent", 0.0),
            mu=data.get("mu", 1.0),
            dt=data.get("dt", 0.005), #0.005
        )


@dataclass
class State:
    t: float = 0.0
    s: float = 0.0
    v: float = 0.0
    a: float = 0.0
    lever_notch: int = 0
    internal_notch: int = 0
    atc_overspeed: bool = False
    finished: bool = False
    stop_error_m: Optional[float] = None
    residual_speed_kmh: Optional[float] = None
    score: Optional[int] = None
    running: bool = False
    paused: bool = False  # 🎮 게임 일시정지 상태

    # ▼ 타이머(카운트다운): float 원본 + 정수 표시값
    time_budget_s: float = 0.0            # 스테이지 부여 시간(초)
    time_remaining_s: float = 0.0         # 남은 시간(초) — 0 아래로 내려갈 수 있음
    timer_enabled: bool = False           # 타이머 사용 여부
    time_remaining_int: int = 0           # 정수 표시용(내림)
    time_overrun_s: float = 0.0           # 초과 시간(양수)
    time_overrun_int: int = 0             # 초과 시간 정수 표시
    time_overrun_started: bool = False    # 오버런 진입 여부


# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------

def build_vref(L: float, a_ref: float):
    def vref(s: float):
        rem = max(0.0, L - s)
        return math.sqrt(max(0.0, 2.0 * a_ref * rem))
    return vref


# ------------------------------------------------------------
# Simulator
# ------------------------------------------------------------

class StoppingSim:
    def __init__(self, veh: Vehicle, scn: Scenario):

       
        self._planned_v0 = scn.v0  # 출발 버튼 누를 때 쓸 예정 속도
        # --- 인계 지점(남은거리) & 히스테리시스 설정 ---
        self.tasc_takeover_rem_m = 250.0 # 인계 거리 (m)
        self.tasc_takeover_hyst_m = 1 # add some randomness like real world tasc beacons!

        self.veh = veh
        self.scn = scn
        self.state = State(t=0.0, s=0.0, v=scn.v0, a=0.0, lever_notch=0, internal_notch=0, finished=False)
        self.running = False
        self.random_mode = False  # Flag to control game-over behavior in Random Scenario mode
        self.final_notch_on_finish = 0  # Store notch when simulation finishes for random mode reload
        self.vref = build_vref(scn.L, 0.8 * veh.a_max)
        self._cmd_queue = deque()

        # 초기 제동(B1/B2) 판정
        self.first_brake_start: Optional[float] = None
        self.first_brake_done: bool = False

        # ▼ 추가: 정확 판정을 위한 상태
        self.first_brake_notch: Optional[int] = None # 1 또는 2로 고정
        self.first_brake_start_t: Optional[float] = None
        self.seen_zero_notch: bool = False # NOTCH_HISTORY[0] == 0 보장용

        # 기록
        self.notch_history: List[int] = []
        self.time_history: List[float] = []

        # EB 사용 여부
        self.eb_used = False
        self.run_over = False
        # 저크 계산
        self.prev_a = 0.0
        self.jerk_history: List[float] = []

        # ---------- TASC ----------``
        self.tasc_enabled = False
        self.tasc_enabled_initially = False  # random mode에서 TASC 복구용 플래그
        self.manual_override = False
        self.tasc_deadband_m = 0.05 #0.05
        self.tasc_hold_min_s = 0.05
        self._tasc_last_change_t = 0.0
        self._tasc_phase = "build"  # "build" → "relax"
        self._tasc_peak_notch = 1
        # 대기/활성 상태
        self.tasc_armed = False
        self.tasc_active = False

        self.tasc_relax_margin_m = test_margin
        # 직전 단계 변경 후 추가로 요구할 최소 홀드시간 (초)
        self.tasc_relax_hold_s = 0

        # μ-저항 분리: rr_factor는 항상 1.0로 고정(μ와 무관)
        self.rr_factor = 1.0

        # ---- 성능 최적화: TASC 예측 캐시/스로틀 ----
        self._tasc_pred_cache = {
            "t": -1.0, "v": -1.0, "notch": -1,
            "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')
        }
        self._tasc_pred_interval = 0.1  # 100ms - 더 효율적인 재계산 간격
        self._tasc_last_pred_t = -1.0
        self._tasc_speed_eps = 0.5  # m/s - 캐시 유효성 범위 확대

        # ---- B5 필요 여부 캐시/스로틀 ----
        self._need_b5_last_t = -1.0
        self._need_b5_last = False
        self._need_b5_interval = 0.05

        
        # -------------------- 동력/응답/상태 --------------------
        self.pwr_accel = 0.0   # 동력 가속도 (forward_notch_accels 반영)
        
        # -------------------- 제동/응답/상태 --------------------
        self.brk_accel = 0.0
        self.brk_elec = 0.0
        self.brk_air  = 0.0



        self.tau_apply = 0.25
        self.tau_release = 0.8
        self.tau_apply_eb = 0.15
        self.tau_release_lowv = 0.8

        self.wsp_state = "normal"
        self.wsp_timer = 0.0

        self._a_cmd_filt = 0.0  # 명령 가속도 1차 필터

        # -------------- 타이머 정책(카운트다운) --------------
        # 표 기반 / 공식 기반 / 보정 기반 자동 산출
        self.timer_use_table = False
        self.timer_table = {}             # 예: {60:35, 70:30, 80:26}
        self.timer_v_target_kmh = 70.0    # 공식 기반 목표 속도(km/h)
        self.timer_buffer_s = 60.0        # 여유초 

        # ---------- 타이머 자동 산출(보정 데이터 기반) ----------
        # 보정 데이터: [{"v":60, "L":200, "t":23}, ...]  (km/h, m, sec)
        # ---------- 타이머 자동 산출(보정 데이터 기반) ----------
# 보정 데이터: [{"v":60, "L":200, "t":23}, ...]  (km/h, m, sec)
        self.timer_calib: List[dict] = [
            {"v": 40, "L": 150, "t": 27},
            {"v": 60, "L": 200, "t": 28},
            {"v": 70, "L": 300, "t": 32},
            {"v": 90, "L": 500, "t": 40},
            {"v": 130, "L": 900, "t": 49}
        ]
        self.timer_idw_power = 2.0         # IDW 거듭제곱
        # 속도/거리 정규화 스케일(거리 계산 공정성 확보)
        self.timer_norm_v = 100.0          # km/h 스케일
        self.timer_norm_L = 300.0          # m 스케일
        # 기준점에서 멀면 공식기반과 블렌딩
        self.timer_blend_threshold = 1.5   # 정규화 거리 기준

        # ▼ 극단값/이상치 처리용 가드레일
        self.timer_min_s = 5.0
        self.timer_max_s = 300.0
        self.timer_min_effective_v_kmh = 12.0   # 공식에 쓰는 최소 유효 속도
        self.timer_max_effective_v_kmh = 110.0  # 공식에 쓰는 최대 유효 속도
        self.timer_far_outlier_scale = 0.35     # 아주 멀면 공식 가중 하한

        # 타임오버 페널티/보너스 정책
        self.timer_overtime_penalty_per_s = 20.0  # 1초당 -20점
        self.timer_overtime_penalty_cap = 400.0   # 최대 페널티
        self.timer_exact_bonus = 100              # 정수 0초 도착 시 +100점

        # 입력 보정 기록(클라이언트에 안내용)
        self.last_input_sanitized = {}

    def _tasc_relax_margin_for_notch(self, notch: int) -> float:
        """
        노치에 따라 동적으로 릴렉스 마진을 반환.
        기본 매핑:
          notch >= 5 -> 10.0 m
          notch >= 4 -> 5.0  m
          notch >= 2 -> 0.5  m
        그 외는 기존 기본값(self.tasc_relax_margin_m)을 사용.
        """
        if notch >= 6:
            return 3.0  #relax margin 릴랙스 마진
        if notch >= 5:
            return 2.0
        if notch >= 4:
            return 1.0
        return float(self.tasc_relax_margin_m)
    # ----------------- Timer helpers -----------------

    def set_timer_calibration(self, points: List[dict],
                              norm_v: float = None,
                              norm_L: float = None,
                              idw_power: float = None,
                              blend_threshold: float = None):
        """보정 표를 통째로 교체"""
        self.timer_calib = []
        for p in points:
            v = float(p.get("v") or p.get("v_kmh"))
            L = float(p.get("L") or p.get("dist"))
            t = float(p.get("t") or p.get("time"))
            self.timer_calib.append({"v": v, "L": L, "t": t})
        if norm_v is not None: self.timer_norm_v = float(norm_v)
        if norm_L is not None: self.timer_norm_L = float(norm_L)
        if idw_power is not None: self.timer_idw_power = float(idw_power)
        if blend_threshold is not None: self.timer_blend_threshold = float(blend_threshold)

    def _idw_predict_time(self, v_kmh: float, L_m: float) -> Tuple[float, float]:
        """보정 표 기반 IDW 추정. (예상시간, 기준점까지의 최소 정규화거리) 반환"""
        if not self.timer_calib:
            return float("nan"), float("inf")
        eps = 1e-6
        num = 0.0
        den = 0.0
        min_d = float("inf")
        for p in self.timer_calib:
            dv = (v_kmh - p["v"]) / max(eps, self.timer_norm_v)
            dL = (L_m   - p["L"]) / max(eps, self.timer_norm_L)
            d = (dv*dv + dL*dL) ** 0.5
            min_d = min(min_d, d)
            w = 1.0 / ((d + eps) ** self.timer_idw_power)
            num += w * p["t"]
            den += w
        t_idw = num / max(eps, den)
        return t_idw, min_d

    def _formula_time(self, v0_kmh: float, L_m: float) -> float:
        """
        기본 공식 기반 시간 = L / v_eff + buffer
        v_eff는 v_target과 v0의 완만한 혼합(안정적 추정).
        """
        v_target = self.timer_v_target_kmh
        # 거리 비율로 혼합 가중(멀수록 v_target 비중 ↑) — 0~1로 스케일
        r = min(1.0, max(0.0, L_m / self.timer_norm_L))  # 300m 기준
        # 너무 느린 v0는 하한, 너무 빠른 v0는 상한
        v0_clip = min(self.timer_max_effective_v_kmh,
                      max(self.timer_min_effective_v_kmh, v0_kmh))
        v_eff = (1.0 - 0.35*r) * v0_clip + (0.35*r) * v_target  # r↑일수록 v_target 쪽
        v_ms = max(0.1, v_eff / 3.6)
        return float(L_m / v_ms + self.timer_buffer_s)

    def _compute_time_budget_auto(self, v_kmh: float, L_m: float) -> float:
        # 1) 보정 표(IDW)ㅋ
        t_idw, min_d = self._idw_predict_time(v_kmh, L_m)

        # 2) 강화된 공식 기반
        t_formula = self._formula_time(v_kmh, L_m)

        # 3) 블렌딩(기준점과 멀수록 공식 비중↑)
        if not self.timer_calib or math.isnan(t_idw):
            t = t_formula
        else:
            if min_d < self.timer_blend_threshold:
                # 기준점 근방 → 보정 표 신뢰
                t = t_idw
            else:
                # 먼 이상치 → 선형으로 공식 비중↑
                alpha = max(0.0, 1.0 - (min_d / (self.timer_blend_threshold * 2.0)))
                # 공식 비중 하한(너무 멀면 공식 최소 35% 반영)
                formula_weight = max(1.0 - alpha, self.timer_far_outlier_scale)
                t = (1.0 - formula_weight) * t_idw + formula_weight * t_formula

        # 4) 최종 클램핑
        t = max(self.timer_min_s, min(self.timer_max_s, t))
        return t

    def _compute_time_budget(self) -> float:
        """스테이지 시작 시 부여할 제한시간(초) 계산"""
        if not self.state.timer_enabled:
            return 0.0

        v0_kmh = self.scn.v0 * 3.6
        L_m = self.scn.L

        # A) 보정 표가 있으면 자동 산출 우선
        if self.timer_calib:
            return self._compute_time_budget_auto(v0_kmh, L_m)

        # B) 정적 테이블 매핑 사용 시
        if self.timer_use_table and self.timer_table:
            v0_round = round(v0_kmh)
            key = min(self.timer_table.keys(), key=lambda k: abs(int(k) - v0_round))
            return float(self.timer_table[key])

        # C) 그 외엔 공식 기반
        return self._formula_time(v0_kmh, L_m)

 
    def _effective_brake_accel(self, notch: int, v: float) -> float:
        #  악셀(음수) 또는 N(0)에서는 '브레이크 없음'
        if notch <= 0:
            return 0.0

        if notch >= len(self.veh.notch_accels):
            return 0.0

        # 기본 제동 가속도(음수)
        base = float(self.veh.notch_accels[notch]) # 음수여야 정상(제동)
        k_srv = 0.85
        k_eb = 0.98
        is_eb = (notch == self.veh.notches - 1)
        k_adh = k_eb if is_eb else k_srv
        a_cap = -k_adh * float(self.scn.mu) * 9.81 # 음수

        a_eff = max(base, a_cap)
        if a_eff <= a_cap + 1e-6:
            scale = 0.90 if v > 8.0 else 0.85
            a_eff = a_cap * scale
        return a_eff

    def _grade_accel(self) -> float:
        return -9.81 * (self.scn.grade_percent / 100.0)

    def _davis_accel(self, v: float) -> float:
        """Davis 저항을 가속도로 환산 (A0/B1/C2는 차량 객체의 최신값 사용) - 최적화"""
        if v < 0.01:  # 매우 낮은 속도에서는 저항 무시
            return 0.0
        A0 = self.veh.A0 * self.rr_factor
        B1 = self.veh.B1 * self.rr_factor
        C2 = self.veh.C2
        v_sq = v * v  # 한 번만 계산
        F = A0 + B1 * v + C2 * v_sq  # N
        return -F / self.veh.mass_kg

    # ----------------- 기타 헬퍼 -----------------

    def _blend_w_regen(self, v: float) -> float:
        """재생 에너지 혼합 비율 (최적화: 3.6 곱셈 1회만)"""
        v_kmh = v * 3.6
        if v_kmh >= 20.0: 
            return 1.0
        if v_kmh <= 8.0:  
            return 0.0
        return (v_kmh - 8.0) / 12.0

    def _update_brake_dyn_split(self, a_total_cmd: float, v: float, is_eb: bool, dt: float):
        w = self._blend_w_regen(v)
        a_cmd_e = a_total_cmd * w
        a_cmd_a = a_total_cmd * (1.0 - w)
        tau_e_apply, tau_e_rel = (0.18, 0.40) if v * 3.6 >= 15 else (0.30, 0.50)
        tau_a_apply, tau_a_rel = (0.45, 0.75) if v * 3.6 < 10 else (0.30, 0.60)
        if is_eb:
            tau_a_apply, tau_a_rel = 0.15, 0.45
        e_stronger = (a_cmd_e < self.brk_elec)
        a_stronger = (a_cmd_a < self.brk_air)
        tau_e = tau_e_apply if e_stronger else tau_e_rel
        tau_a = tau_a_apply if a_stronger else tau_a_rel
        self.brk_elec += (a_cmd_e - self.brk_elec) * (dt / max(1e-6, tau_e))
        self.brk_air  += (a_cmd_a - self.brk_air ) * (dt / max(1e-6, tau_a))
        self.brk_accel = self.brk_elec + self.brk_air

    def _wsp_update(self, v: float, a_demand: float, dt: float):
        a_cap = -0.85 * self.scn.mu * 9.81
        margin = 0.05
        if self.wsp_state == "normal":
            if a_demand < (a_cap - margin) and v * 3.6 > 3.0:
                self.wsp_state = "release"
                self.wsp_timer = 0.12
                return min(a_demand, 0.5 * a_cap)
            return a_demand
        elif self.wsp_state == "release":
            self.wsp_timer -= dt
            if self.wsp_timer <= 0.0:
                self.wsp_state = "reapply"
                self.wsp_timer = 0.15
            return min(a_demand, 0.3 * a_cap)
        else:
            self.wsp_timer -= dt
            if self.wsp_timer <= 0.0:
                self.wsp_state = "normal"
            return min(a_demand, 0.8 * a_cap)

    # ----------------- Controls -----------------
    # safe-guard for notch limits
    def _clamp_notch(self, n: int) -> int:
        # forward_notches 길이만큼 음수 허용
        min_notch = -len(self.veh.forward_notch_accels)  # 예: -2
        max_notch = len(self.veh.notch_accels) - 2       # EB 직전 (W/S로는 EB 도달 불가)
        return max(min_notch, min(max_notch, n))


    def queue_command(self, name: str, val: int = 0):
        # Emergency brake should bypass command latency (tau_cmd) and
        # apply immediately (real-world behaviour). For EB we also
        # shortcut the brake filter by setting brake split and
        # _a_cmd_filt to the commanded brake so the strong brake
        # takes effect without waiting tau_brk.
        # if name == "setInternalNotch":
        #     # Internal notch changes are immediate and do not queue
        #     cmd = {"t": self.state.t, "name": name, "val": val}
        #     self._apply_command(cmd)
        #     return
        if name == "atcOverspeed":
            self.state.atc_overspeed = bool(val)
            return

        if name == "emergencyBrake":
            # Apply immediately
            cmd = {"t": self.state.t, "name": name, "val": val}
            self._apply_command(cmd)

            # If Emergency Brake was set, force brake states to commanded values
            try:
                st = self.state
                if st.lever_notch == (self.veh.notches - 1):
                    a_cmd_total = self._effective_brake_accel(st.lever_notch, st.v)
                    w = self._blend_w_regen(st.v)
                    # Split into electric/air immediately to skip slower tau_brk
                    self.brk_elec = a_cmd_total * w
                    self.brk_air  = a_cmd_total * (1.0 - w)
                    self.brk_accel = self.brk_elec + self.brk_air
                    # Bypass tau_brk filter so _a_cmd_filt equals commanded now
                    self._a_cmd_filt = a_cmd_total
            except Exception:
                # Be defensive: if anything goes wrong, fall back to queued behavior
                self._cmd_queue.append({"t": self.state.t + self.veh.tau_cmd, "name": name, "val": val})
            return

        # Normal commands respect command latency
        # However, when the train is essentially stopped, apply manual
        # notch changes immediately so the player can depart without
        # waiting for the command latency to elapse (fixes frozen-input
        # feel after soft-reset/advanceStation).
        try:
            st = self.state
            speed_m_s = float(getattr(st, 'v', 0.0))
        except Exception:
            speed_m_s = 0.0

        immediate_apply = False
        if name in ("stepNotch", "applyNotch", "setNotch", "release"):
            # if stopped (or almost stopped) -> immediate
            if speed_m_s <= 0.05:
                immediate_apply = True

        if immediate_apply:
            # apply immediately but DO NOT queue a duplicate entry
            # to avoid double-applying the command (fixes double-notch bug)
            cmd = {"t": self.state.t, "name": name, "val": val}
            self._apply_command(cmd)
            return

        self._cmd_queue.append({"t": self.state.t + self.veh.tau_cmd, "name": name, "val": val})

    def _apply_command(self, cmd: dict):
        st = self.state
        name = cmd["name"]
        val = cmd["val"]

        # # ▼ TASC가 'active'인 상태에서 수동 개입이 들어오면 즉시 TASC를 OFF
        # if self.tasc_enabled and self.tasc_active and name in ("emergencyBrake"):
        #     self.tasc_enabled = False
        #     self.tasc_active = False
        #     self.tasc_armed = False
        #     if DEBUG:
        #         print("[TASC] manual intervention while ACTIVE -> TASC OFF")

        # ▼ 이하 기존 로직(lever_notch 직접 조작) 유지
        if name == "stepNotch":
            old_notch = st.lever_notch
            st.lever_notch = self._clamp_notch(st.lever_notch + val)
            if DEBUG:
                print(f"Applied stepNotch: {old_notch} -> {st.lever_notch}")
        elif name == "release":
            st.lever_notch = 0
        elif name == "emergencyBrake":
            st.lever_notch = self.veh.notches - 1
            if st.v > 0:
                self.eb_used = True
        elif name == "setNotch":
            st.lever_notch = self._clamp_notch(val)
        elif name == "setInternalNotch":
            st.internal_notch = self._clamp_notch(val)

        # When a forward notch is applied while stopped, compute and apply the proper
        # acceleration immediately so the filter is initialized correctly for smooth
        # acceleration from rest.
        try:
            if st.lever_notch < 0 and st.v == 0.0:
                # Compute what the power accel should be at v=0
                pwr = self.compute_power_accel(st.lever_notch, 0.0)
                # Initialize the filter to this value so acceleration starts smoothly
                if pwr > 0:
                    self._a_cmd_filt = pwr
                    if DEBUG:
                        print(f"[APPLY_CMD] Forward notch at v=0: initialized _a_cmd_filt={self._a_cmd_filt:.3f} m/s² (notch={st.lever_notch})")
        except Exception as e:
            if DEBUG:
                print(f"[APPLY_CMD] Error initializing accel filter: {e}")
            pass


    # ----------------- Lifecycle -----------------

    def reset(self):
        # ▼ 기존 상태의 timer_enabled를 보존(없으면 False)
        prev_timer_enabled = getattr(self.state, "timer_enabled", False)
        # ▼ 기존 running 상태를 보존 (UI 명령이 random mode 상태 변경 시 중단되지 않도록)
        prev_running = getattr(self, "running", False)
        # ▼ Random mode에서 notch 보존: 이전 실행의 final_notch_on_finish를 사용
        # (setInitial 호출 시 random_mode=true이고 이전 시뮬레이션이 finished되었으면 notch를 유지)
        # 첫 번째 run에서는 final_notch_on_finish=0이므로 다시 0에서 시작(정상)
        # 두 번째 이후 run에서는 이전 final_notch_on_finish값으로 시작(보존된 notch)
        if self.random_mode and hasattr(self, 'final_notch_on_finish'):
            prev_lever_notch = int(self.final_notch_on_finish)
            # EB는 보존하지 않음 - 다음 run에서는 일반 제동으로 변환 (최대 normal notch)
            max_normal_notch = self.veh.notches - 2  # EB 직전
            if prev_lever_notch >= self.veh.notches - 1:  # EB인 경우
                prev_lever_notch = max_normal_notch  # EB를 최대 일반 노치로 변환
                if DEBUG:
                    print(f"[RESET] EB detected in final_notch_on_finish, converting to max normal notch ({max_normal_notch})")
            if DEBUG:
                print(f"[RESET] *** RANDOM MODE NOTCH PRESERVATION: Using final_notch_on_finish={prev_lever_notch}")
        else:
            prev_lever_notch = 0
            if DEBUG:
                print(f"[RESET] Normal reset: lever_notch starting at 0 (random_mode={self.random_mode})")

        # 계획 속도는 시나리오의 v0를 따로 들고 있고, 대기 상태에는 v=0으로 둔다
        self._planned_v0 = self.scn.v0

        self.state = State(
            t=0.0, s=0.0, v=0.0, a=0.0, lever_notch=prev_lever_notch, finished=False
        )
        
        # Only stop if not in a continuing random scenario
        # (If running due to random mode, keep it running unless explicitly stopped)
        if not self.random_mode:
            self.running = False
        else:
            self.running = prev_running
        self._cmd_queue.clear()

        self.first_brake_start = None
        self.first_brake_done = False
        self.first_brake_notch = None
        self.first_brake_start_t = None
        self.seen_zero_notch = False
        self.eb_used = False
        self.notch_history.clear()
        self.time_history.clear()

        self.prev_a = 0.0
        self.jerk_history = []

        # self.manual_override = False
        self._tasc_last_change_t = 0.0
        if not self.tasc_active:
            self._tasc_phase = "build"
            self._tasc_peak_notch = 1

        self.tasc_active = False
        self.tasc_armed = bool(self.tasc_enabled)

        self._tasc_pred_cache.update({
            "t": -1.0, "v": -1.0, "notch": -1,
            "s_cur": float('inf'), "s_up": float('inf'), "s_dn": float('inf')
        })
        self._tasc_last_pred_t = -1.0

        self._need_b5_last_t = -1.0
        self._need_b5_last = False

        self.brk_accel = 0.0
        self.brk_elec = 0.0
        self.brk_air  = 0.0

        self.wsp_state = "normal"
        self.wsp_timer = 0.0

        self._a_cmd_filt = 0.0

        self.rr_factor = 1.0

        # ▼ 보존해 둔 타이머 플래그 복원
        self.state.timer_enabled = prev_timer_enabled

        # ▼ 타이머 초기화 (예산시간/남은시간 세팅)
        if self.state.timer_enabled:
            self.state.time_budget_s = self._compute_time_budget()
            self.state.time_remaining_s = self.state.time_budget_s
        else:
            self.state.time_budget_s = 0.0
            self.state.time_remaining_s = 0.0

        # ▼ 정수 표시 초기화
        self.state.time_remaining_int = math.floor(self.state.time_remaining_s)
        self.state.time_overrun_s = 0.0
        self.state.time_overrun_int = 0
        self.state.time_overrun_started = False

        if DEBUG:
            print(f"Simulation reset | timer_enabled={self.state.timer_enabled} "
                  f"| budget={self.state.time_budget_s:.2f}s | L={self.scn.L} v0={self.scn.v0*3.6:.1f}km/h")

    def start(self):
        self.reset()
        self.state.v = float(self._planned_v0) 
        self.running = True
        self._t_start = time.time()  # sim_loop에서 참조 가능
        if DEBUG:
            print("Simulation started")


    def compute_power_accel(self, lever_notch: int, v: float) -> float:
        """
        동력 가속도 계산:
        - 음수 notch(P1~Pn)만 동력, 0 이상은 0
        - Vehicle에 T_max_kN, P_max_kW가 설정되어 있으면
          '출력 제한 + 견인력 제한' 모델 사용
        - 없으면 예전 방식(노치별 고정 a + 속도 fade)으로 fallback
        """
        # 전진 노치가 아니면(=제동 또는 N) 가속 없음
        if lever_notch >= 0:
            return 0.0
            
        if self.veh.T_max_kN > 0.0 and self.veh.P_max_kW > 0.0:
            
            # 1. 노치 인덱스 처리
            n_forward = len(self.veh.forward_notch_accels) or self.veh.forward_notches
            n_forward = max(1, n_forward)
            idx = int(abs(lever_notch)) - 1
            idx = max(0, min(idx, n_forward - 1))

            # ------------------------------------------------------------------
            # [Core] 토크(Torque) vs 출력(Power) 비율 분리 (핵심!)
            # ------------------------------------------------------------------
            if self.veh.maxSpeed_kmh >= 200: 
                # A. 토크 비율 (저속 힘): 님께서 튜닝한 '초반 몰빵' 값 유지

                torque_table = [
                    0.40, 0.48, 0.56, # P1~P3 : 출발/저속에서 강하게
                    0.64, 0.70, 0.76, # P4~P6
                    0.82, 0.88, 0.92, # P7~P9
                    0.95, 0.98, 0.99, 1.00 # P10~P13
                ]

                power_table = [
                    0.10, 0.16, 0.24, # P1~P3 : 저속에서 출력은 제한적이지만 현실적으로 더 크게
                    0.34, 0.45, 0.58, # P4~P6 : 중저속 구간에 힘 보강
                    0.70, 0.80, 0.88, # P7~P9
                    0.94, 0.98, 0.995, 1.00 # P10~P13 : 최상단
                ]

                       
                # 안전하게 값 가져오기
                safe_idx = min(idx, len(torque_table)-1)
                ratio_T = torque_table[safe_idx]
                ratio_P = power_table[safe_idx]

            else:
                # 일반열차 (선형)
                ratio_T = (idx + 1) / n_forward
                ratio_P = (idx + 1) / n_forward

            # ------------------------------------------------------------------
            # [Physics] 이원화된 물리량 제한 적용
            # ------------------------------------------------------------------
            P_max_W = self.veh.P_max_kW * 1000.0
            T_max_N = self.veh.T_max_kN * 1000.0
            mass_kg = self.veh.mass_kg
            v_safe = max(0.1, v) 

            # 1. 노치별 토크 한계 (저속 영역 결정)
            # P1이라도 T_max의 30%를 쓰므로 아주 강력함
            F_torque_limit = T_max_N * ratio_T

            # 2. 노치별 출력 한계 (고속 영역 결정)
            # P1은 P_max의 2%만 쓰므로, 속도가 오르면 F = P/v 에 의해 힘이 급격히 소멸
            F_power_limit = (P_max_W * ratio_P) / v_safe

            # 3. 최종 물리 견인력 (둘 중 작은 값)
            F_physics = min(F_torque_limit, F_power_limit)

            # ------------------------------------------------------------------
            # [Adhesion & Finalize] 점착 및 보정
            # ------------------------------------------------------------------
            v_kmh = v * 3.6
        # ------------------------------------------------------------------
        # [Speed Cap] 노치별 속도 한계 (여기가 핵심!)
        # ------------------------------------------------------------------
        # 각 노치가 힘을 낼 수 있는 최대 속도를 정의합니다.
        # 예: P1은 40km/h 넘어가면 힘이 빠짐, P13은 330km/h까지 힘을 냄
        
        # 1. 노치별 한계 속도 테이블 (차량 특성에 맞게 튜닝 필요)
        # 예시: 총 13단이라고 가정할 때 (저단은 낮게, 고단은 높게)
        # 비율(0.0 ~ 1.2) * 최고속도(maxSpeed_kmh) 로 계산하거나 직접 입력
        
            if self.veh.type == "고속":
                # 고속열차 (HEMU, KTX 등) 노치별 속도 제한 비율
                # P1~P4: 저속/구내 운전 (정밀)
                # P5~P8: 중속/간선 운전
                # P9~P13: 고속선 운전 (P13 = 100% 성능)
                limit_ratios = [
                    # --- 저속 구간 (정밀 제어, 연결/분리/서행) ---
                    0.05,  # P1 :  18 km/h
                    0.18,  # P2 :  60 km/h
                    0.30,  # P3 : 108 km/h
                    0.40,  # P4 : 144 km/h
                    
                    # --- 중속 크루징 (일반선/터널 등 속도 유지용) ---
                    0.50,  # P5 : 180 km/h (Target Match)
                    0.58,  # P6 : 209 km/h
                    0.66,  # P7 : 238 km/h
                    0.74,  # P8 : 266 km/h (Target Match)
                    
                    # --- 고속 주행 (공기저항을 이겨내기 위한 고출력 구간) ---
                    0.80,  # P9 : 288 km/h
                    0.86,  # P10: 310 km/h (300km/h 정속 주행용)
                    0.92,  # P11: 331 km/h
                    0.97,  # P12: 349 km/h
                    1.00   # P13: 360 km/h (설계 최고속도, 오버파워 없이 100% 출력)
                ]
            else:
                # 일반 열차 (기존 로직 유지)
                limit_ratios = [(i + 1) / n_forward * 1.2 for i in range(n_forward)]

            # 안전하게 인덱스 가져오기
            safe_limit_idx = min(idx, len(limit_ratios) - 1)
            notch_max_speed = self.veh.maxSpeed_kmh * limit_ratios[safe_limit_idx]
            
# ... (Existing code for limit_ratios calculation) ...

            # 안전하게 인덱스 가져오기
            safe_limit_idx = min(idx, len(limit_ratios) - 1)
            notch_max_speed = self.veh.maxSpeed_kmh * limit_ratios[safe_limit_idx]
            
            # 2. 페이드 아웃 (Fade-out) 처리 개선
            # P1, P2 (저속/정밀 제어)와 나머지 노치(주행)의 거동을 분리합니다.
            
            if idx <= 1: # idx 0 is P1, idx 1 is P2
                # [CASE A: 저속 정밀 구간 (P1~P2)]
                # 목표: 오버슈트 없이 부드럽게 한계 속도에 안착하거나 멈추기 위함.
                # cutoff_range: 속도 한계에 가까워질 때 힘을 빼기 시작하는 범위 (작게 설정하여 정밀도 향상)
                # min_residual: 한계 속도 도달 시 남길 힘 (0.0에 가깝게 하여 과속 방지)
                cutoff_range = 15.0  
                min_residual = 0.05  
            else:
                # [CASE B: 일반/고속 주행 구간 (P3~P13)]
                # 목표: 공기 저항을 이기고 속도를 유지(Cruising)하거나 가속하기 위함.
                # cutoff_range: 고속에서는 관성이 크므로 미리 힘을 조절하기 위해 넓게 잡음 (40km/h)
                # min_residual: 고속 주행 시 공기저항 상쇄를 위해 일정 힘 유지 (0.4)
                cutoff_range = 40.0
                min_residual = 0.4

            # ... (Proceed with the calculation using cutoff_range and min_residual) ...

            # 2. 로직 적용
            if v_kmh > notch_max_speed:
                # [수정됨] 0으로 끄지 않고, 계산된 힘의 5%만 찔끔 남겨둠
                F_physics = F_physics * min_residual
                
            elif v_kmh > (notch_max_speed - cutoff_range):
                # [수정됨] 100% -> 5%로 부드럽게 이어지도록 보간(Interpolation)
                
                # 구간 내 진행률 (0.0: 진입 ~ 1.0: 한계도달)
                progress = (v_kmh - (notch_max_speed - cutoff_range)) / cutoff_range
                
                # 1.0 에서 min_residual 까지 줄어드는 계수 계산
                # 예: progress가 0.5(중간)면 힘은 약 52.5% 발휘
                factor = 1.0 - (progress * (1.0 - min_residual))
                
                F_physics *= factor
            else:
                # 한계 속도 한참 전: 100% 온전한 힘
                pass

            # ------------------------------------------------------------------
            # [Finalize] 가속도 변환
            # ------------------------------------------------------------------
            # 여기서 계산된 F_physics는 순수 견인력이므로
            # 나중에 바깥에서 a_davis(저항)를 빼주면 자연스럽게 평형 속도가 맞춰짐
            
            a_pwr = F_physics / mass_kg

            # 극저속 보정 (기존 로직)
            if v_kmh < 5.0 and idx >= 0:
                a_pwr = max(a_pwr, 0.15 * (ratio_T / 0.30)) 

            if self.veh.a_max > 0:
                a_pwr = min(a_pwr, self.veh.a_max)

            return a_pwr
            
            # F_final = F_physics

            # # 가속도 산출
            # a_pwr = F_final / mass_kg

            # # 극저속(5km/h 미만) 기동성 보정 (P1 이상일 때만)
            # if v_kmh < 5.0 and idx >= 0:
            #     # ratio_T(토크비율)을 사용하여 묵직한 출발 보장
            #     a_pwr = max(a_pwr, 0.15 * (ratio_T / 0.30)) 

            # if self.veh.a_max > 0:
            #     a_pwr = min(a_pwr, self.veh.a_max)

            # return a_pwr

        # ------------------------
        # 2) 물리 파라미터 없으면 기존 방식 유지 여기서 단 조절!!!
        # ------------------------
        # Legacy fallback: per-notch base accel with realistic fade-by-speed
        n_notches = len(self.veh.forward_notch_accels)
        idx = max(0, min(-lever_notch - 1, n_notches - 1))
        base_accel = float(self.veh.forward_notch_accels[idx])

        v_kmh = v * 3.6
        max_v_kmh = max(1.0, float(self.veh.maxSpeed_kmh))

        # 노치별 plateau 종료, exponential 시작, min_factor
        flat_ends   = [5.0, 20.0, 32.0, 33.0, 33.0]         # plateau 끝
        exp_starts  = [30.0, 33.0, 54.0, 54.0, 54.0]       # exponential 시작
        min_factors = [0.0003, 0.04, 0.07, 0.14, 0.27]  # 노치별 최솟값

        # idx가 배열 범위를 넘어가면 마지막 값 사용
        s_k = flat_ends[min(idx, len(flat_ends)-1)]
        e_k = exp_starts[min(idx, len(exp_starts)-1)]
        min_factor = min_factors[min(idx, len(min_factors)-1)]

        # mid_factor는 linear 구간 중간값
        mid_factor = 0.35 + 0.1 * idx
        mid_factor = min(1.0, max(min_factor, mid_factor))

        # ----- Region 1: plateau -----
        if v_kmh <= s_k:
            factor = 1.0

        # ----- Region 2: linear decay -----
        elif v_kmh <= e_k:
            t = (v_kmh - s_k) / max(1e-6, (e_k - s_k))
            factor = 1.0 - (1.0 - mid_factor) * t
            factor = max(factor, min_factor)  # linear에서도 min_factor 보장

        # ----- Region 3: exponential tail -----
        else:
            t = (v_kmh - e_k) / max(1e-6, (max_v_kmh - e_k))
            factor = (mid_factor - min_factor) * (2.71828 ** (-3 * t)) + min_factor
            factor = max(factor, min_factor)

        factor = min(1.0, factor)  # 상한 1.0
        return base_accel * factor


    def eb_used_from_history(self) -> bool:
        return any(n == self.veh.notches - 1 for n in self.notch_history)

    # ------ stopping distance helpers ------

    def _estimate_stop_distance(self, notch: int, v0: float) -> float:
        # if notch <= 0:
        #     return float('inf')



        dt = 0.03
        v = max(0.0, v0)
        a = float(self.state.a)
        s = 0.0



        brk_elec = float(self.brk_elec)
        brk_air  = float(self.brk_air)
        wsp_state = self.wsp_state
        wsp_timer = float(self.wsp_timer)
        a_cmd_filt = float(self._a_cmd_filt)

        rem_now = self.scn.L - self.state.s
        limit = float(rem_now + 8.0)

        ctrl_delay = max(self._tasc_pred_interval, self.tasc_hold_min_s)
        latency_margin = v * ctrl_delay

        for _ in range(2400):

                                # 동력 가속도 계산 (전진 notch)
            if notch < 0:  # P1~P5
                pwr_accel = self.compute_power_accel(notch, v)
            else:
                pwr_accel = 0.0
            is_eb = (notch == self.veh.notches - 1)
            a_cmd_total = self._effective_brake_accel(notch, v)

            w = self._blend_w_regen(v)
            a_cmd_e = a_cmd_total * w
            a_cmd_a = a_cmd_total * (1.0 - w)

            tau_e_apply, tau_e_rel = (0.18, 0.40) if v * 3.6 >= 15 else (0.30, 0.50)
            tau_a_apply, tau_a_rel = (0.45, 0.75) if v * 3.6 < 10 else (0.30, 0.60)
            if is_eb:
                tau_a_apply, tau_a_rel = 0.15, 0.45

            e_stronger = (a_cmd_e < brk_elec)
            a_stronger = (a_cmd_a < brk_air)

            tau_e = tau_e_apply if e_stronger else tau_e_rel
            tau_a = tau_a_apply if a_stronger else tau_a_rel

            brk_elec += (a_cmd_e - brk_elec) * (dt / max(1e-6, tau_e))
            brk_air  += (a_cmd_a - brk_air ) * (dt / max(1e-6, tau_a))
            a_brake = brk_elec + brk_air

            a_cap = -0.85 * self.scn.mu * 9.81
            margin = 0.05
            if wsp_state == "normal":
                if a_brake < (a_cap - margin) and v * 3.6 > 3.0:
                    wsp_state = "release"
                    wsp_timer = 0.12
                    a_brake = min(a_brake, 0.5 * a_cap)
            elif wsp_state == "release":
                wsp_timer -= dt
                if wsp_timer <= 0.0:
                    wsp_state = "reapply"
                    wsp_timer = 0.15
                a_brake = min(a_brake, 0.3 * a_cap)
            else:
                wsp_timer -= dt
                if wsp_timer <= 0.0:
                    wsp_state = "normal"
                    wsp_timer = 0.0
                a_brake = min(a_brake, 0.8 * a_cap)

            a_grade = self._grade_accel()
            a_davis = self._davis_accel(v)
            a_target = pwr_accel + a_brake + a_grade + a_davis

            if notch == 0:
                a_target = self._grade_accel() + self._davis_accel(v)

            # E233계열은 회생제동 우선 제어 방식을 사용하며, 속도 약 7~10 km/h 이하에서 회생제동이 실질적으로 사라집니다.

            # 이때 공기제동이 완전히 takeover
            # 공기압 밸브 제어에 따른 지연이 필연적으로 존재합니다.
            # 일반적으로 응답상수 τ ≈ 0.5초 내외로 알려져 있습니다.

            # 반면 회생제동의 경우 전류 제어 응답이 수백 ms(0.2~0.3s) 수준이라
            # 체감상 약 1.5~2배 느리다고 볼 수 있습니다.

            # 제동력 자체는 저속 시 마찰제동의 압력 제한 및 마찰계수 변화로 인해
            # 약 0.7~0.8배 수준으로 감소합니다.

            # # (신규) 속도 기반 소프트 스톱
            rem_pred = max(0.0, rem_now - s)
            # v_kmh = v * 3.6
            # if v_kmh <= soft_stop_di and notch > 0:
            #     alpha = max(0.0, min(1.0, v_kmh / soft_stop_di))
            #     a_soft = (-0.30) * alpha + (soft_stop_const) * (1.0 - alpha)
            #     w_soft = 1.0 - alpha
            #     a_target = (1.0 - w_soft) * a_target + w_soft * a_soft

            ### NEW NEW NEW
            # if notch == 1 or rem_pred <= 0.0:
            #     a_target = min(a_target, 0.0)

            # # 응답시간(tau_brk)을 저속에서 늘려서 밸브 지연 반영
            # v_kmh_local = v * 3.6
            # effective_tau_brk = self.veh.tau_brk * 1.5 if v_kmh_local <= air_brake_vi else self.veh.tau_brk

            # a_cmd_filt += (a_target - a_cmd_filt) * (dt / max(1e-6, effective_tau_brk))
            ### NEW NEW NEW

            if notch == 1 or rem_pred <= 0.0:
                a_target = min(a_target, 0.0)

            a_cmd_filt += (a_target - a_cmd_filt) * (dt / max(1e-6, self.veh.tau_brk))

            max_da = self.veh.j_max * dt
            v_kmh = v * 3.6
            if v_kmh <= 5.0:
                scale = 0.25 + 0.75 * (v_kmh / 5.0)
                max_da *= scale

            da = a_cmd_filt - a
            if da > max_da:
                da = max_da
            elif da < -max_da:
                da = -max_da
            a += da

            v = max(0.0, v + a * dt)
            s += v * dt + 0.5 * a * dt * dt

            if v <= 0.01:
                break
            if s > limit:
                break

        return s + latency_margin

    def _stopping_distance(self, notch: int, v: float) -> float:
        if notch <= 0:
            return float('inf')
        return self._estimate_stop_distance(notch, v)

    def _tasc_predict(self, cur_notch: int, v: float):
        """TASC 정지거리 예측 (최적화된 캐싱)"""
        st = self.state
        need = False
        # 캐시 유효성 검사 - 간격 기반
        if (st.t - self._tasc_last_pred_t) >= self._tasc_pred_interval:
            need = True
        # 속도 변화 감지
        if abs(v - self._tasc_pred_cache["v"]) >= self._tasc_speed_eps:
            need = True
        # 노치 변화 감지
        if cur_notch != self._tasc_pred_cache["notch"]:
            need = True
        
        # 캐시 유효 - 재계산 스킵
        if not need:
            return (
                self._tasc_pred_cache["s_cur"],
                self._tasc_pred_cache["s_up"],
                self._tasc_pred_cache["s_dn"],
            )

        # 필요한 경우에만 계산 (100ms마다 최대 1회)
        max_normal_notch = self.veh.notches - 2
        s_cur = self._stopping_distance(cur_notch, v) if cur_notch > 0 else float("inf")
        s_up = self._stopping_distance(cur_notch + 1, v) if cur_notch + 1 <= max_normal_notch else 0.0
        s_dn = self._stopping_distance(cur_notch - 1, v) if cur_notch - 1 >= 1 else float("inf")

        # 캐시 업데이트
        self._tasc_pred_cache.update(
            {"t": st.t, "v": v, "notch": cur_notch, "s_cur": s_cur, "s_up": s_up, "s_dn": s_dn}
        )
        self._tasc_last_pred_t = st.t
        return s_cur, s_up, s_dn

    def _need_B5_now(self, v: float, remaining: float) -> bool:
        st = self.state
        if (st.t - self._need_b5_last_t) < self._need_b5_interval and self._need_b5_last_t >= 0.0:
            return self._need_b5_last

        v0_kmh = self.scn.v0 * 3.6
        if v0_kmh < 75.0:
            n_ref = 2
        elif v0_kmh < 85.0:
            n_ref = 3
        elif v0_kmh < 95.0:
            n_ref = 3
        else:
            n_ref = 4

        s_ref = self._stopping_distance(n_ref, v)
        need = s_ref > (remaining + self.tasc_deadband_m)
        self._need_b5_last = need
        self._need_b5_last_t = st.t
        return need

    # ----------------- Main step -----------------

    def step(self):
        st = self.state
        dt = self.scn.dt

        while self._cmd_queue and self._cmd_queue[0]["t"] <= st.t:
            self._apply_command(self._cmd_queue.popleft())
        
        # if self.notch_history[-1] != st.lever_notch:
        if st.v > 0.1:
            self.notch_history.append(st.lever_notch)

        self.time_history.append(st.t)

        # If train has finished (come to complete stop), freeze all dynamics
        if st.finished:
            st.a = 0.0
            st.v = max(0.0, st.v)  # Ensure v doesn't go negative
            # Do not update position or time, skip rest of physics
            return
        # --- 기존 first_brake_done 로직 삭제하고 아래로 교체 ---
        # 초기 0단을 한번이라도 봤는지 표시
        if not self.seen_zero_notch and st.lever_notch == 0:
            self.seen_zero_notch = True

        if not self.first_brake_done and self.seen_zero_notch and not self.tasc_active:
            cur = st.lever_notch
            prev = self.notch_history[-2] if len(self.notch_history) >= 2 else None

            if self.first_brake_notch is None:
                # 0 → (1|2)로 '처음' 진입했을 때만 스타트
                if prev == 0 and cur in (1, 2):
                    self.first_brake_notch = cur
                    self.first_brake_start_t = st.t
            else:
                # 같은 단을 유지해야 함 (1↔2 스위칭도 NG)
                if cur == self.first_brake_notch:
                    if (st.t - self.first_brake_start_t) >= 0.999: # float 여유
                        self.first_brake_done = True
                else:
                    # 1초 채우기 전 이탈(0, 3이상, 전진단, 또는 1↔2 변경) → 리셋
                    self.first_brake_notch = None
                    self.first_brake_start_t = None

        # ▼ 타이머(카운트다운): 0 아래로도 계속 진행 - 최적화
        if st.timer_enabled and not st.finished:
            st.time_remaining_s -= dt
            # 정수 표시값은 0.01초마다만 업데이트 (불필요한 계산 감소)
            st.time_remaining_int = int(st.time_remaining_s)
            if st.time_remaining_s < 0.0 and not st.time_overrun_started:
                st.time_overrun_s = -st.time_remaining_s
                st.time_overrun_int = abs(st.time_remaining_int)
                st.time_overrun_started = True
                st.issues = getattr(st, "issues", {})
                st.issues["timeout_started"] = True

        # ---------- TASC ----------
        if self.tasc_enabled and not st.finished:
            dwell_ok = (st.t - self._tasc_last_change_t) >= self.tasc_hold_min_s
            rem_now = self.scn.L - st.s
            cur = st.internal_notch
            max_normal_notch = self.veh.notches - 2

            if self.tasc_armed and not self.tasc_active:
                takeover_on = self.tasc_takeover_rem_m
                hyst = self.tasc_takeover_hyst_m
                if rem_now <= (takeover_on + hyst):
                    self.tasc_active = True
                    self.tasc_armed = False
                    self._tasc_last_change_t = st.t

            if self.tasc_active:
                if not self.first_brake_done:
                    self.first_brake_done = True
                else:
                    s_cur, s_up, s_dn = self._tasc_predict(cur, st.v)
                    changed = False
                    if self._tasc_phase == "build":
                        if cur < max_normal_notch and s_cur > (rem_now - self.tasc_deadband_m):
                            if dwell_ok:
                                st.internal_notch = self._clamp_notch(cur + 1)
                                self._tasc_last_change_t = st.t
                                self._tasc_peak_notch = max(self._tasc_peak_notch, st.internal_notch)
                                changed = True
                        else:
                            self._tasc_phase = "relax"
                    if self._tasc_phase == "relax" and not changed:
                        # if cur > 1 and s_dn <= (rem_now + self.tasc_deadband_m):
                        #     if dwell_ok:
                        #         st.internal_notch = self._clamp_notch(cur - 1)
                        #         self._tasc_last_change_t = st.t
                        if cur > 1:
                            target_notch = cur - 1
                            # 변경: "릴렉스되어 내려갈 목표 노치(target_notch)가 4 이상"일 때만 마진/홀드를 적용
                            # (즉, 5→4, 6→5 처럼 결과가 여전히 4 이상인 경우에만 지연)
                            if cur >= test_notch:
                                # 더 보수적으로 완화하려면 추가 마진 요구
                                margin = self._tasc_relax_margin_for_notch(cur)
                                relax_allowed = (s_dn <= (rem_now + self.tasc_deadband_m - margin))
                                time_since_change = st.t - self._tasc_last_change_t
                                if relax_allowed and dwell_ok and (time_since_change >= self.tasc_relax_hold_s):
                                    st.internal_notch = self._clamp_notch(target_notch)
                                    self._tasc_last_change_t = st.t
                            else:
                                # 목표 노치가 3 이하(3,2,1 등)면 기존 즉시 완화 규칙 유지
                                if s_dn <= (rem_now + self.tasc_deadband_m) and dwell_ok:
                                    st.internal_notch = self._clamp_notch(target_notch)
                                    self._tasc_last_change_t = st.t
        # ---------- Dynamics ----------

        # internal_notch가 더 높으면 그것을 사용
        effective_notch = max(st.lever_notch, st.internal_notch)
        
        # ATC 오버스피드 시에만 조건부 계산
        if self.state.atc_overspeed:
            pwr_accel = self.compute_power_accel(effective_notch, st.v)
        else:
            pwr_accel = self.compute_power_accel(st.lever_notch, st.v)

        a_cmd_brake = self._effective_brake_accel(effective_notch, st.v)
        is_eb = (effective_notch == self.veh.notches - 1)
        self._update_brake_dyn_split(a_cmd_brake, st.v, is_eb, dt)
        a_brake = self._wsp_update(st.v, self.brk_accel, dt)
        a_grade = self._grade_accel()
        a_davis = self._davis_accel(st.v)

        # 최종 가속도 계산 (순차 누적)
        a_target = pwr_accel + a_brake + a_grade + a_davis

        rem_now = self.scn.L - st.s
        v_kmh = st.v * 3.6

        if effective_notch >= 1:
            a_target = min(a_target, 0.0)

        # 가속도 필터 (tau_brk 사용)
        tau_inv = dt / max(1e-6, self.veh.tau_brk)
        self._a_cmd_filt += (a_target - self._a_cmd_filt) * tau_inv

        # 저크 제한 (jerk limiting)
        max_da = self.veh.j_max * dt
        # 저속/브레이크 상황에서 저크 제한 완화
        if v_kmh <= 5.0 and effective_notch >= 1:
            scale = 0.25 + 0.75 * (v_kmh / 5.0)
            max_da *= scale

        # 가속도 변화 클램핑
        da = self._a_cmd_filt - st.a
        if da > max_da:
            da = max_da
        elif da < -max_da:
            da = -max_da
        st.a += da

        # 물리 적분
        st.v = max(0.0, st.v + st.a * dt)
        st.s += st.v * dt + 0.5 * st.a * dt * dt
        st.t += dt

        # ---------- Finish ----------
        rem = self.scn.L - st.s
        if not st.finished and (rem <= -5.0 or (rem <= 1.0 and st.v <= 0.0)):
            st.finished = True
            st.stop_error_m = self.scn.L - st.s
            st.residual_speed_kmh = st.v * 3.6

            score = 0
            st.issues = {}

            if self.eb_used:
                score -= 500
                st.issues["unnecessary_eb_usage"] = True

            if not self.first_brake_done:
                score -= 100
            else:
                score += 300

            last_notch = self.notch_history[-1] if self.notch_history and abs(st.stop_error_m) <= 1.0 else 0
            if last_notch in [0, 1, 2]:
                score += 300
                st.issues["stop_not_b1"] = False
                st.issues["stop_not_b1_msg"] = "정차 시 승차감 양호"
            # elif last_notch == 0:
            #     score -= 100
            #     st.issues["stop_not_b1"] = True
            #     st.issues["stop_not_b1_msg"] = "정차 시 N으로 정차함 - 열차 미끄러짐 주의"
            else:
                score -= 100
                st.issues["stop_not_b1"] = True
                st.issues["stop_not_b1_msg"] = "정차 시 B3 이상으로 정차함 - 승차감 불쾌"

            if self.is_stair_pattern(self.notch_history):
                score += 300
            else:
                if self.tasc_enabled and not self.manual_override:
                    score += 300

            err_abs = abs(st.stop_error_m or 0.0) # 정차 오차 절대값
            error_score = max(0, 500 - int(err_abs / 2 * 500))
            score += error_score

            if abs(st.stop_error_m or 0.0) < 0.01: # 정차 오차 0.1cm 이내
                score += 500

            st.issues["early_brake_too_short"] = not self.first_brake_done
            st.issues["step_brake_incomplete"] = not self.is_stair_pattern(self.notch_history)
            st.issues["stop_error_m"] = st.stop_error_m

            jerk = abs((st.a - self.prev_a) / dt)
            self.prev_a = st.a
            self.jerk_history.append(jerk)

            avg_jerk, jerk_score = self.compute_jerk_score()
            score += int(jerk_score)

            # ▼ 타임오버/정확 도착 보정 (정수 초 기준)
            # if st.timer_enabled:
            #     score += 100

                # 남은 시간이 양수(조기 도착)인 경우는 보너스/페널티 없음
            # if st.timer_enabled:
            #     # 정수 0초(내림) 도착 → 보너스
            #     if st.time_remaining_int == 0:
            #         score += int(self.timer_exact_bonus)
            #         st.issues["timer_exact_hit"] = True
            #         st.issues["timer_exact_bonus"] = int(self.timer_exact_bonus)
            #         st.time_overrun_int = 0
            #         st.time_overrun_s = 0.0
            #     elif st.time_remaining_int < 0:
            #         over_s = abs(st.time_remaining_int)  # 정수 초
            #         overtime_pen = min(
            #             self.timer_overtime_penalty_per_s * over_s,
            #             self.timer_overtime_penalty_cap
            #         )
            #         score -= int(overtime_pen)
            #         st.issues["timeout_overrun_s"] = over_s
            #         st.issues["timeout_penalty"] = int(overtime_pen)
            #     # 남은 시간이 양수(조기 도착)인 경우는 보너스/페널티 없음

            if self.run_over: #안전 위반에 대한 강력한 패널티
                score -= 1000

            minq = 300
            maxq = 1200
            if score < minq:
                score = minq    
            
            if score > maxq:
                score = maxq

            norm = (score - minq) / (maxq - minq) * 100
            norm = max(0, min(100, norm))
            score = round(norm, 0)
            st.score = score
            
            # Store final notch for random mode reload
            self.final_notch_on_finish = st.lever_notch
            
            # Random mode에서 TASC가 초기에 활성화되었다면 복구
            if self.random_mode and self.tasc_enabled_initially:
                self.tasc_enabled = True
                self.tasc_armed = True
                self.tasc_active = False
                self._tasc_phase = "build"
                self._tasc_peak_notch = 1
                if DEBUG:
                    print("[FINISH] TASC restored for next run (random_mode + tasc_enabled_initially)")
            
            # In Random Scenario mode, keep running=True so physics can continue after finish
            # (waiting for advanceStation command). In normal mode, stop the simulation.
            if not self.random_mode:
                self.running = False
            if DEBUG:
                print(f"Avg jerk: {avg_jerk:.4f}, jerk_score: {jerk_score:.2f}, final score: {score}")
                print(f"Simulation finished: stop_error={st.stop_error_m:.3f} m, score={score}")
                print(f"[FINISH] Preserving final notch: {self.final_notch_on_finish} (random_mode={self.random_mode})")

    def remove_negative_values(self, notches: List[int]) -> List[int]:
        """마지막 음수 값 뒤에 있는 모든 수 반환, 음수가 없으면 원본 리스트 반환"""
        # 역순으로 탐색해서 마지막 음수 값을 찾음
        for i in range(len(notches) - 1, -1, -1):
            if notches[i] < 0:
                # 마지막 음수 이후의 모든 값들 반환
                return notches[i+1:]
        # 음수가 없으면 원본 리스트 반환
        return notches
    
    def remove_adjacent_duplicates(self, lst):
        if not lst:
            return []
        
        result = [lst[0]]
        for x in lst[1:]:
            if x != result[-1]:
                result.append(x) 
        return result
    
    def is_stair_pattern(self, notches: List[int]) -> bool:

        notches = self.remove_adjacent_duplicates(notches)
        # print(notches)
        notches = self.remove_negative_values(notches)
        # print(notches)
        if len(notches) < 5:
            return False

        peak_reached = False
        prev = notches[0]

        for cur in notches[1:]:
            if not peak_reached:
                if cur < prev:  # 내려가기 시작하면 피크 도달
                    peak_reached = True
            else:
                if cur > prev:  # 피크 이후 다시 올라가면 실패
                    return False
            prev = cur

        # 마지막은 1, 2로 끝나야 함
        if notches[-1] not in [1, 2]:
            return False

        return True

    def compute_jerk_score(self):
        dt = self.scn.dt
        window_time = 1.0
        n = int(window_time / dt)
        recent_jerks = self.jerk_history[-n:] if len(self.jerk_history) >= n else self.jerk_history
        if not recent_jerks:
            return 0.0, 0

        avg_jerk = sum(recent_jerks) / len(recent_jerks)
        high_jerk_count = sum(1 for j in recent_jerks if j > 30)
        penalty_factor = min(1, high_jerk_count / 10)
        adjusted_jerk = avg_jerk * (1 + penalty_factor)

        # 12 이하 → 500점, 12~30 → 선형 감소, 30 이상 → 0점
        if adjusted_jerk <= 10:
            return adjusted_jerk, 0
        
        low_bound = 21.6
        high_bound = 24
        if adjusted_jerk <= low_bound:
            jerk_score = 500
        elif adjusted_jerk <= high_bound:
            jerk_score = 500 * (high_bound - adjusted_jerk) / (high_bound - low_bound)  # 500 * (30 - jerk)/18
        else:
            jerk_score = 0

        return adjusted_jerk, jerk_score


    def snapshot(self):
        st = self.state
        return {
            "t": round(st.t, 3),
            "server_ts": time.time(),  # 보간용 서버 타임스탬프
            "s": st.s,
            "v": st.v,
            "a": st.a,
            "lever_notch": st.lever_notch,
            "remaining_m": self.scn.L - st.s,
            "L": self.scn.L,
            "v_ref": self.vref(st.s),
            "finished": st.finished,
            "stop_error_m": st.stop_error_m,
            "residual_speed_kmh": st.v * 3.6,
            "running": self.running,
            "grade_percent": self.scn.grade_percent,
            "grade": self.scn.grade_percent,
            "score": getattr(st, "score", 0),
            "issues": getattr(st, "issues", {}),
            "tasc_enabled": getattr(self, "tasc_enabled", False),
            "tasc_armed": getattr(self, "tasc_armed", False),
            "tasc_active": getattr(self, "tasc_active", False),
            "train_name": self.veh.name,
            "maxSpeed_kmh": self.veh.maxSpeed_kmh,

            # # HUD/디버그용 (업데이트된 Davis 확인 가능)
            # "mu": float(self.scn.mu),
            # "rr_factor": float(self.rr_factor),
            # "davis_A0": self.veh.A0,
            # "davis_B1": self.veh.B1,
            # "davis_C2": self.veh.C2,

            # # ▼ 타이머 표시용
            # "timer_enabled": st.timer_enabled,
            # "time_budget_s": st.time_budget_s,
            # "time_remaining_s": st.time_remaining_s,     # float 원본(음수 가능)
            # "time_remaining_int": st.time_remaining_int, # 정수 표시(내림)
            # "time_overrun_s": st.time_overrun_s,
            # "time_overrun_int": st.time_overrun_int,
            # "time_overrun_started": st.time_overrun_started,

            # # 입력 보정 정보(서버 클램프)
            # "input_sanitized": getattr(self, "last_input_sanitized", {}),
        }


# ------------------------------------------------------------
# FastAPI app
# ------------------------------------------------------------

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")

app = FastAPI()

# /static 경로 제공
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
#yes
@app.get("/")
async def root():
    return HTMLResponse(open(os.path.join(STATIC_DIR, "index.html"), "r", encoding="utf-8").read())

@app.get("/favicon.ico")
async def favicon():
    return FileResponse(os.path.join(STATIC_DIR, "favicon.ico"))


@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    cur_length = 10
    cur_load_rate = 0.70
    vehicle_json_path = os.path.join(STATIC_DIR, "emu_db/e233_1000.json")
    scenario_json_path = os.path.join(BASE_DIR, "scenario.json")

    vehicle = Vehicle.from_json(vehicle_json_path)
    vehicle.notch_accels = list(reversed(vehicle.notch_accels))
    vehicle.notches = len(vehicle.notch_accels)

    scenario = Scenario.from_json(scenario_json_path)

    sim = StoppingSim(vehicle, scenario)
    sim.reset() # 초기화

    #  최초 연결 직후: 기본 편성/탑승률을 즉시 반영해 총중량/데이비스 재계산
    sim.veh.update_mass(cur_length) # 1차: 편성 반영

    base_1c_t = sim.veh.mass_t # 1량(공차) 톤
    pax_1c_t = 10.5 # 1량 만차 탑승 톤 (가정값)
    total_tons = cur_length * (base_1c_t + pax_1c_t * cur_load_rate)

    sim.veh.mass_kg = total_tons * 1000.0
    sim.veh.recompute_davis(sim.veh.mass_kg)
    sim.veh.calibrate_C2_from_power(300.0, eta=0.85)

    if DEBUG:
        print(f"[INIT] len={cur_length}, load={cur_load_rate*100:.1f}% "
              f"-> base_1c_t={base_1c_t:.3f} t, pax_1c_t={pax_1c_t:.2f} t "
              f"| total={total_tons:.2f} t, mass_kg={sim.veh.mass_kg:.0f} "
              f"| A0={sim.veh.A0:.1f}, B1={sim.veh.B1:.2f}, C2={sim.veh.C2:.2f}")

    sim.reset() #  재계산 반영된 상태로 다시 초기화(처음부터 일관)
    sim.running = False


    # 전송 속도: 60Hz (더 부드러운 애니메이션)
    send_interval = 1.0 / 20.0

    # ---- 분리된 비동기 루프들 ----
    async def recv_loop():
        # vehicle(바깥 스코프 변수)에 재할당 가능하게
        nonlocal vehicle, cur_length, cur_load_rate
        try:
            while True:
                msg = await ws.receive_text()
                try:
                    data = json.loads(msg)
                except Exception:
                    if DEBUG:
                        print("Invalid JSON received.")
                    continue

                if data.get("type") != "cmd":
                    continue

                payload = data.get("payload", {})
                name = payload.get("name")

                if name == "setInitial":
                    speed = payload.get("speed")
                    dist = payload.get("dist")
                    grade = payload.get("grade", 0.0) / 10.0
                    mu = float(payload.get("mu", 1.0))
                    random_mode = payload.get("random_mode", False)
                    if speed is not None and dist is not None:
                        # ▼ 서버 측 이중 방어(클램프) — 프론트와 동일
                        v_kmh_raw = float(speed)
                        L_raw = float(dist)
                        v_kmh = max(0,  min(300.0, v_kmh_raw))
                        L_m   = max(150.0, min(60000.0,  L_raw))

                        sim.scn.v0 = v_kmh / 3.6
                        sim.scn.L = L_m
                        sim.scn.grade_percent = float(grade)
                        sim.scn.mu = mu
                        sim.random_mode = bool(random_mode)

                        # 클램프 여부 기록
                        sim.last_input_sanitized = {
                            "speed_input": v_kmh_raw, "speed_used": v_kmh,
                            "dist_input": L_raw, "dist_used": L_m,
                            "clamped": (v_kmh != v_kmh_raw) or (L_m != L_raw)
                        }

                        if DEBUG:
                            print(f"setInitial: v0={v_kmh:.1f}km/h ({v_kmh_raw}), "
                                  f"L={L_m:.0f}m ({L_raw}), grade={grade}%, mu={mu}, random_mode={random_mode}")
                        sim.reset()  # reset()이 timer_enabled 보존 + budget 재계산

                elif name == "advanceStation":
                    # Advance to the next station. Make this robust by performing
                    # a light reset while preserving world coordinate so visuals
                    # remain continuous.
                    try:
                        # In random mode, allow advance even if not finished yet
                        # In normal mode, only allow after finished
                        is_random_mode = getattr(sim, 'random_mode', False)
                        is_finished = getattr(sim.state, 'finished', False)
                        
                        if not is_random_mode and not is_finished:
                            # only meaningful when previous run has finished (in normal mode)
                            if DEBUG:
                                print(f"[ADVANCE] Game not finished yet, ignoring advanceStation (not in random mode)")
                            continue
                        
                        if is_random_mode and not is_finished and DEBUG:
                            print(f"[ADVANCE] Random mode: allowing advance even though game not finished")

                        dist = float(payload.get('dist', 600.0))
                        grade = float(payload.get('grade', 0.0)) / 10.0
                        mu = float(payload.get('mu', sim.scn.mu))

                        # clamp sensible ranges (same policy as setInitial)
                        dist = max(150.0, min(60000.0, dist))

                        # preserve state across soft-reset
                        prev_s = float(sim.state.s)
                        prev_timer_enabled = getattr(sim.state, 'timer_enabled', False)
                        # Preserve current notch - use the last notch the player set
                        # Priority: use notch_history if available, otherwise final_notch_on_finish, otherwise current lever_notch
                        if sim.notch_history:
                            prev_lever_notch = int(sim.notch_history[-1])
                        elif is_finished and getattr(sim, 'final_notch_on_finish', None) is not None:
                            prev_lever_notch = int(sim.final_notch_on_finish)
                        else:
                            prev_lever_notch = int(sim.state.lever_notch)

                        if DEBUG:
                            print(f"[ADVANCE] Starting soft reset: prev_s={prev_s:.2f}, timer_enabled={prev_timer_enabled}, notch={prev_lever_notch} (from notch_history={len(sim.notch_history)} entries), is_finished={is_finished}")

                        # perform a reset to clear command queue / timing artifacts,
                        # then restore the world coordinate and apply new scenario end
                        sim.reset()

                        # restore preserved flags/position/notch
                        sim.state.s = prev_s
                        sim.state.timer_enabled = prev_timer_enabled
                        sim.state.lever_notch = prev_lever_notch

                        # set new absolute L so that remaining == dist
                        sim.scn.L = float(prev_s) + dist
                        sim.scn.grade_percent = float(grade)
                        sim.scn.mu = float(mu)

                        # recompute timer budget according to new scenario
                        try:
                            tb = sim._compute_time_budget()
                        except Exception:
                            tb = getattr(sim.state, 'time_budget_s', 0.0)

                        sim.state.time_budget_s = float(tb)
                        sim.state.time_remaining_s = float(tb)
                        sim.state.time_remaining_int = math.floor(sim.state.time_remaining_s)

                        # enable timer if a positive budget was computed
                        if sim.state.time_budget_s > 0.0:
                            sim.state.timer_enabled = True

                        # refresh vref in case L changed
                        try:
                            sim.vref = build_vref(sim.scn.L, 0.8 * sim.veh.a_max)
                        except Exception:
                            pass

                        # start from rest and clear finished/run_over
                        sim.state.finished = False
                        sim.state.stop_error_m = None
                        sim.state.residual_speed_kmh = 0.0
                        sim.state.v = 0.0
                        sim.state.a = 0.0
                        # NOTE: Do NOT reset lever_notch here - keep the preserved notch from previous run
                        # sim.state.lever_notch = 0

                        # CRITICAL: Set running=True to ensure physics loop continues
                        sim.running = True
                        sim.run_over = False
                        
                        # Ensure acceleration filter is reset to allow clean start
                        sim._a_cmd_filt = 0.0

                        if DEBUG:
                            print(f"[ADVANCE] Completed: s={prev_s:.2f}m, L={sim.scn.L:.0f}m (remaining={dist:.0f}m), grade={sim.scn.grade_percent}%, mu={sim.scn.mu:.2f}, timer={sim.state.time_budget_s:.1f}s")
                            print(f"[ADVANCE] *** CRITICAL CHECK ***")
                            print(f"[ADVANCE] >>> sim.running={sim.running} (should be True)")
                            print(f"[ADVANCE] >>> sim.state.v={sim.state.v} (should be 0.0)")
                            print(f"[ADVANCE] >>> sim.state.finished={sim.state.finished} (should be False)")
                            print(f"[ADVANCE] >>> sim.state.lever_notch={sim.state.lever_notch} (should be {prev_lever_notch}, preserved from previous run)")
                            print(f"[ADVANCE] >>> sim._a_cmd_filt={sim._a_cmd_filt} (will be initialized when notch applied)")
                    except Exception as e:
                        if DEBUG:
                            print(f"[ADVANCE] ERROR: {e}")
                        import traceback
                        traceback.print_exc()

                elif name == "start":
                    sim.start()
                    sim.run_over = False

                elif name in ("stepNotch", "applyNotch"):
                    delta = int(payload.get("delta", 0))
                    sim.queue_command("stepNotch", delta)
                    # Update final_notch_on_finish if simulation is finished (for random mode)
                    # Process pending commands first to get actual notch value
                    if sim.state.finished:
                        while sim._cmd_queue and sim._cmd_queue[0]["t"] <= sim.state.t:
                            cmd = sim._cmd_queue.popleft()
                            sim._apply_command(cmd)
                        sim.final_notch_on_finish = sim.state.lever_notch
                        if DEBUG:
                            print(f"[FINISHED NOTCH] Updated to {sim.final_notch_on_finish} (stepNotch delta={delta})")

                elif name == "release":
                    sim.queue_command("release", 0)
                    # Update final_notch_on_finish if simulation is finished
                    if sim.state.finished:
                        while sim._cmd_queue and sim._cmd_queue[0]["t"] <= sim.state.t:
                            cmd = sim._cmd_queue.popleft()
                            sim._apply_command(cmd)
                        sim.final_notch_on_finish = 0
                        if DEBUG:
                            print(f"[FINISHED NOTCH] Updated to 0 (release)")

                elif name == "emergencyBrake":
                    sim.queue_command("emergencyBrake", 0)
                    # Update final_notch_on_finish if simulation is finished
                    if sim.state.finished:
                        while sim._cmd_queue and sim._cmd_queue[0]["t"] <= sim.state.t:
                            cmd = sim._cmd_queue.popleft()
                            sim._apply_command(cmd)
                        sim.final_notch_on_finish = sim.state.lever_notch
                        if DEBUG:
                            print(f"[FINISHED NOTCH] Updated to {sim.state.lever_notch} (EB)")

                elif name == "setNotch":
    # 'val'이나 'delta'에 상관없이 value가 있다면 우선
                    val = payload.get("val", payload.get("delta", payload.get("value", 0)))
                    sim.queue_command("setNotch", val)
                    # Update final_notch_on_finish if simulation is finished
                    if sim.state.finished:
                        while sim._cmd_queue and sim._cmd_queue[0]["t"] <= sim.state.t:
                            cmd = sim._cmd_queue.popleft()
                            sim._apply_command(cmd)
                        sim.final_notch_on_finish = sim.state.lever_notch
                        if DEBUG:
                            print(f"[FINISHED NOTCH] Updated to {sim.state.lever_notch} (setNotch val={val})")
                elif name == "setInternalNotch":
                    val = payload.get("val", payload.get("delta", payload.get("value", 0)))
                    sim.queue_command("setInternalNotch", val)
                elif name == "atcOverspeed":
                    val = payload.get("val", payload.get("delta", payload.get("value", 0)))
                    sim.queue_command("atcOverspeed", val)
                elif name == "setGrade":
                    # Random grade update from client
                    grade = float(payload.get("grade", 0.0))
                    sim.scn.grade_percent = grade
                    if DEBUG:
                        print(f"[RANDOM GRADE] Updated to {grade}% (‰: {grade * 10:.1f})")

                elif name == "setTrainLength":
                    length = int(payload.get("length", 8))
                    cur_length = length #  상태 저장

                    # 길이 반영
                    sim.veh.update_mass(cur_length)

                    # 탑승률이 이미 있다면 총중량 덮어쓰기 + 재계산
                    base_1c_t = sim.veh.mass_t
                    pax_1c_t = 10.5
                    total_tons = cur_length * (base_1c_t + pax_1c_t * cur_load_rate)
                    sim.veh.mass_kg = total_tons * 1000.0
                    sim.veh.recompute_davis(sim.veh.mass_kg)
                    sim.veh.calibrate_C2_from_power(300.0, eta=0.85)

                    if DEBUG:
                        print(f"[Length] {cur_length} cars | load={cur_load_rate*100:.1f}% "
                            f"-> mass_kg={sim.veh.mass_kg:.0f}, A0={sim.veh.A0:.1f}, B1={sim.veh.B1:.2f}, C2={sim.veh.C2:.2f}")
                    sim.reset()

                elif name == "setMassTons":
                    mass_tons = float(payload.get("mass_tons", 200.0))
                    sim.veh.mass_t = mass_tons / int(payload.get("length", 8))
                    sim.veh.mass_kg = mass_tons * 1000.0
                    sim.veh.recompute_davis(sim.veh.mass_kg) #  새 질량으로 재계산
                    sim.veh.calibrate_C2_from_power(300.0, eta=0.85)
                    if DEBUG:
                        print(
                            f"총중량={mass_tons:.2f} t -> "
                            f"A0={sim.veh.A0:.1f}, B1={sim.veh.B1:.2f}, C2={sim.veh.C2:.2f}"
                        )
                    sim.reset()

                elif name == "setLoadRate":
                    cur_load_rate = float(payload.get("loadRate", 0.0)) / 100.0 #  상태 저장

                    # 길이/탑승률로 총중량 재산출
                    base_1c_t = sim.veh.mass_t
                    pax_1c_t = 10.5
                    total_tons = cur_length * (base_1c_t + pax_1c_t * cur_load_rate)

                    sim.veh.update_mass(cur_length) # 1차 (길이 반영)
                    sim.veh.mass_kg = total_tons * 1000.0 # 실제 총중량 덮어쓰기
                    sim.veh.recompute_davis(sim.veh.mass_kg) # 최종 재계산
                    sim.veh.calibrate_C2_from_power(300.0, eta=0.85)

                    if DEBUG:
                        print(f"[LoadRate] length={cur_length}, load={cur_load_rate*100:.1f}% "
                            f"-> mass_kg={sim.veh.mass_kg:.0f}, A0={sim.veh.A0:.1f}, B1={sim.veh.B1:.2f}, C2={sim.veh.C2:.2f}")
                    sim.reset()



                elif name == "setTASC":
                    enabled = bool(payload.get("enabled", False))
                    sim.tasc_enabled = enabled
                    sim.tasc_enabled_initially = enabled  # random mode 복구용 저장
                    if enabled:
                        sim.manual_override = False
                        sim._tasc_last_change_t = sim.state.t
                        sim._tasc_phase = "build"
                        sim._tasc_peak_notch = 1
                        sim.tasc_armed = True
                        sim.tasc_active = False
                    if DEBUG:
                        print(f"TASC set to {enabled}")

                elif name == "obstacleStopSuccess":
                    sim.eb_used = False
                    sim.first_brake_done = True
                elif name == "obstacleStopFail":
                    sim.run_over = True


                elif name == "setMu":
                    value = float(payload.get("value", 1.0))
                    sim.scn.mu = value
                    if DEBUG:
                        print(f"마찰계수(mu)={value}")
                    sim.reset()


                elif name == "setVehicleFile":
                    rel = payload.get("file", "")
                    if rel:
                        try:
                            # 경로 정규화
                            rel_norm = rel.strip()
                            if rel_norm.startswith("/static/emu_db/"): rel_norm = rel_norm[len("/static/emu_db/"):]
                            elif rel_norm.startswith("static/emu_db/"): rel_norm = rel_norm[len("static/emu_db/"):]
                            path = os.path.join(STATIC_DIR + "/emu_db", rel_norm)

                            if not os.path.isfile(path):
                                raise FileNotFoundError(path)

                            newv = Vehicle.from_json(path)
                            newv.notch_accels = list(reversed(newv.notch_accels))
                            newv.notches = len(newv.notch_accels)
                            newv.recompute_davis(newv.mass_kg)

                            sim.veh = newv
                            vehicle = newv

                            # 🔒 차량 교체 직후, 현재 길이/탑승률 재적용 (순서 무관 일관성 보장)
                            sim.veh.update_mass(cur_length)
                            base_1c_t = sim.veh.mass_t
                            pax_1c_t = 10.5
                            total_tons = cur_length * (base_1c_t + pax_1c_t * cur_load_rate)
                            sim.veh.mass_kg = total_tons * 1000.0
                            sim.veh.recompute_davis(sim.veh.mass_kg)
                            sim.veh.calibrate_C2_from_power(300.0, eta=0.85)

                            sim.reset()

                            if DEBUG:
                                print(f"[Vehicle] switched -> {rel} ({path}) | len={cur_length}, load={cur_load_rate*100:.1f}% "
                                    f"| mass_kg={sim.veh.mass_kg:.0f} A0={sim.veh.A0:.1f} B1={sim.veh.B1:.2f} C2={sim.veh.C2:.2f}")
                        except Exception as e:
                            if DEBUG: print(f"[Vehicle] load failed: {rel} -> {e}")


                elif name == "reset":
                    sim.reset()

                # ---------- 타이머/페널티/보너스/보정 설정 ----------
                elif name == "setTimerFormula":
                    # payload: { "enabled": true, "v_target_kmh": 70, "buffer_s": 0 }
                    sim.timer_use_table = False
                    sim.state.timer_enabled = bool(payload.get("enabled", True))
                    sim.timer_v_target_kmh = float(payload.get("v_target_kmh", 70))
                    sim.timer_buffer_s = float(payload.get("buffer_s", 0.0))
                    sim.reset()

                elif name == "setTimerTable":
                    # payload: { "enabled": true, "table": { "60":35, "70":30, "80":26 } }
                    tbl = payload.get("table", {})
                    sim.timer_use_table = True
                    sim.state.timer_enabled = bool(payload.get("enabled", True))
                    sim.timer_table = {int(k): float(v) for k, v in tbl.items()}
                    sim.reset()

                elif name == "toggleTimer":
                    # payload: { "enabled": false }
                    sim.state.timer_enabled = bool(payload.get("enabled", False))
                    sim.reset()

                elif name == "setTimerPenalty":
                    # payload: { "per_s": 20, "cap": 400 }
                    sim.timer_overtime_penalty_per_s = float(payload.get("per_s", 20.0))
                    sim.timer_overtime_penalty_cap = float(payload.get("cap", 400.0))

                elif name == "setTimerExactBonus":
                    # payload: {"bonus": 100}
                    sim.timer_exact_bonus = float(payload.get("bonus", 100))

                elif name == "setTimerCalib":
                    # payload 예시:
                    # {
                    #   "points":[
                    #     {"v":60, "L":200, "t":23},
                    #     {"v":70, "L":200, "t":28},
                    #     {"v":90, "L":400, "t":30}
                    #   ],
                    #   "norm_v": 100, "norm_L": 300,
                    #   "idw_power": 2.0, "blend_threshold": 1.5
                    # }
                    pts = payload.get("points", [])
                    sim.set_timer_calibration(
                        points=pts,
                        norm_v=payload.get("norm_v"),
                        norm_L=payload.get("norm_L"),
                        idw_power=payload.get("idw_power"),
                        blend_threshold=payload.get("blend_threshold"),
                    )
                    # 자동 산출이 적용되도록 리셋
                    sim.state.timer_enabled = True
                    sim.reset()
                
                elif name == "pause":
                    # 🎮 게임 일시정지
                    sim.state.paused = True
                    if DEBUG:
                        print(f"[PAUSE] Game paused at t={sim.state.t:.2f}s, v={sim.state.v*3.6:.1f}km/h")

                elif name == "resume":
                    # 🎮 게임 재개
                    sim.state.paused = False
                    if DEBUG:
                        print(f"[RESUME] Game resumed from t={sim.state.t:.2f}s, v={sim.state.v*3.6:.1f}km/h")
                
                else:
                     cmd_val = payload.get("val", payload.get("delta", 0))
                     sim.queue_command(name, cmd_val)    
                

        except WebSocketDisconnect:
            if DEBUG:
                print("WebSocket disconnected (recv_loop).")
        except asyncio.CancelledError:
            pass
        except Exception as e:
            if DEBUG:
                print(f"Error during receive: {e}")

    async def sim_loop():
        dt = sim.scn.dt
        step_count = 0
        t_start = None  # 시작 시점은 start() 눌렀을 때 설정
        was_running = False
        was_finished = False
        was_paused = False
        loop_iterations = 0

        while True:
            loop_iterations += 1
            
            # Detect if finished state just changed (soft-reset/advanceStation happened)
            is_finished_now = getattr(sim.state, 'finished', False)
            if is_finished_now != was_finished:
                if DEBUG:
                    print(f"[SIM_LOOP] Finished state changed: {was_finished} → {is_finished_now}")
                if not is_finished_now and was_finished and sim.running:
                    # Just transitioned from finished→not-finished while running
                    # This means advanceStation reset the state, so reset timing!
                    if DEBUG:
                        print(f"[SIM_LOOP] *** DETECTED SOFT-RESET: Resetting timing (iteration {loop_iterations})")
                    t_start = time.time()
                    step_count = 0
                was_finished = is_finished_now
            
            # 🎮 게임 일시정지 상태 확인
            is_paused_now = getattr(sim.state, 'paused', False)
            if is_paused_now and not was_paused:
                if DEBUG:
                    print(f"[SIM_LOOP] Game paused (iteration {loop_iterations})")
            elif not is_paused_now and was_paused:
                if DEBUG:
                    print(f"[SIM_LOOP] Game resumed (iteration {loop_iterations})")
                # 일시정지에서 복귀하면 시간 기준점을 갱신
                t_start = time.time()
                step_count = 0
            was_paused = is_paused_now
            
            if sim.running and not is_paused_now:  # 게임 실행 중이고 일시정지 아님
                if not was_running:
                    if DEBUG:
                        print(f"[SIM_LOOP] Transitioned to running state (iteration {loop_iterations})")
                    t_start = time.time()
                    step_count = 0

                t_now = time.time()
                expected_steps = int((t_now - t_start) / dt)

            # 누적된 스텝만큼만 진행
                for _ in range(step_count, expected_steps):
                    if DEBUG and loop_iterations % 100 == 0:
                        print(f"[SIM_LOOP] Executing step (iteration {loop_iterations}, step {step_count})")
                    sim.step()

                step_count = expected_steps
                was_running = True

            else:
            # ★ 정지 상태에서는 기준값들을 항상 초기화
                if was_running and DEBUG:
                    print(f"[SIM_LOOP] Transitioned to stopped state (iteration {loop_iterations})")
                was_running = False
                t_start = None
                step_count = 0

            await asyncio.sleep(dt)  # dt 기반 sleep (CPU 효율성)


    async def send_loop():
        try:
            while True:
                await ws.send_text(json.dumps({"type": "state", "payload": sim.snapshot()}))
                await asyncio.sleep(send_interval)
        except WebSocketDisconnect:
            if DEBUG:
                print("WebSocket disconnected (send_loop).")
        except asyncio.CancelledError:
            pass
        except Exception as e:
            if DEBUG:
                print(f"Error during send: {e}")

    tasks = [
        asyncio.create_task(recv_loop()),
        asyncio.create_task(sim_loop()),
        asyncio.create_task(send_loop()),
    ]

    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    finally:
        for t in tasks:
            t.cancel()
        try:
            await ws.close()
        except RuntimeError:
            pass