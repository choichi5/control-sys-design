# ✅ Control System Design Final Term Project

**팀원:**  
- 2020741010 최치혁  
- 2020741050 한승엽  

---

## 📚 1. 시스템 선정 (System Selection)
이번 프로젝트에서는 DC 모터를 동역학 시스템으로 선정했습니다.
- **선정 이유:**
  - 로봇 실험 및 모터제어 수업을 통해 실제 적용 가능한 제어 이론을 실습해보고 검증하기 위함.
  - 단순한 구조이면서 상태공간 모델화 및 실제 시뮬레이션 검증이 용이한 DC 모터를 선택했습니다.

---

## 🧮 2. 운동 방정식 (Equations of Motion)
모터의 운동 방정식:
```
τ = Km * i
Vemf = Kb * ω
J * (dω/dt) = -Kf * ω + Km * i
(di/dt) = -(R/L) * i - (Kb/L) * ω + (1/L) * Vapp
```
**직접 정의한 파라미터:**
- R = 2 Ω, L = 0.5 H
- Km = Kb = 0.015
- J = 0.02 kg·m²
- Kf = 0.2 N·m·s

---

## 📐 3. 상태공간 모델 (State Space Model)
```
dx/dt = A * x + B * u
y = C * x + D * u

A = [ -R/L   -Kb/L;
       Km/J  -Kf/J ]
B = [1/L; 0]
C = [0 1]
D = [0]
```
코드(`ControlSystemFinal_ChoiChiHyuk.m`)에 직접 정의함.

---

## 🎯 4. 성능 지표 (Performance Index)
- 감쇠비 ζ = 0.5 이상
- 상승 시간 2초
- 자연 주파수 약 1 rad/s
- 세틀링 타임 9.2초
- 최대 오버슈트 ≤ 16%

> 이 지표를 기반으로 desired poles 및 컨트롤러 파라미터를 결정했습니다.

---

## ✅ 5. 제어 가능성 (Controllability)
```
Controllability matrix = [B, A*B]
Rank 확인 후 결과: 제어 가능 (controllable)
```
직접 계산 및 코드 출력 완료.

---

## ⚙️ 6. 상태 피드백 제어기 설계 (State Feedback Controller)
### (a) Ackermann 기반 설계
- 설계 목표 극점: 감쇠비 및 자연 진동수를 반영해 설정
- `acker(A, B, desired_poles)` 함수로 피드백 게인 K 산출
- Nx, Nu 계산 후 N_b 보상값 적용
- 시뮬레이션: `week_4.slx`

> **이론:** Ackermann's Formula는 제어기가 원하는 극점 위치를 갖도록 K를 설계하는 대표적인 해법으로, 컨트롤러의 성능을 직접 결정할 수 있습니다.

### (b) Symmetric Root Locus 기반 설계
- `-s` 대입 후 대칭 시스템 생성
- 루트로커스 기반 gain (1450) 결정 및 극점 산출
- acker 적용
- `estimator_syme.slx`에서 overshoot 없이 추종 성공.

> **이론:** Symmetric Root Locus는 복잡한 시스템의 극점 배치를 시각화 및 조정 가능하게 하여 보다 직관적 설계가 가능합니다.

---

## 👁️ 7. 관측기 (Observer) 설계
### (a) Full-Order Estimator
- 관측 가능 행렬 O 생성 후 rank 확인 → observable
- 추정기 극점: 제어기 극점의 10배 빠르게 설정
- `L = acker(A', C', desired_estimator_poles)'` 적용
- `estimator.slx`에서 구현 및 시뮬레이션 확인.

### (b) Symmetric Root Locus 기반 추정기
- 대칭 시스템을 활용한 추정기 설계
- 10배 빠른 극점 위치로 설정 및 Ackermann 적용
- `estimator_syme.slx` 시뮬레이션으로 안정적 추정 확인.

> **이론:** 상태 관측기는 측정 불가능한 상태변수를 추정하는 장치이며, 추정기 극점은 실제 시스템 극점보다 더 빠르게 설정해 추정 정확성을 확보합니다.

---

## 🔗 8. Autonomous Estimator 설계
- Non-zero reference input 대응을 위한 M 게인 및 Nx, Nu 적용
- 자율 추정기 루프 구성 및 `estimator_autonomous.slx` 시뮬레이션 구현
- 외란 및 ramp 입력에도 robust한 추종성 확보.

> **이론:** Autonomous Estimator는 비영점 입력 상황에서 안정적인 steady-state tracking을 유지할 수 있도록 설계됩니다.

---

## ➕ 9. 적분 제어기 (Integral Controller)
- 상태공간 확장:
```
A_aug = [A    0;
        -C    0]
```
- 느린 극점(-10) 추가 후 Ackermann 방식으로 Kx_i, Ki 계산
- `estimator_integrator.slx`, `estimator_integrator_syme.slx` 사용

> **이론:** 적분 제어기는 steady-state error를 제거하며 외란과 모델 불확실성에 강인성을 더합니다.

---

## 🏆 10. 최적 조합 및 결론
- **최적 조합:** Desired pole 기반 상태 피드백 + Autonomous Estimator + 적분 제어기
- 오차 최소화, 빠른 응답성, 외란 저항성 및 참조 추종성 모두 확보.

**외란 테스트:**
- Sine & Step 외란 → 강인한 추종성 유지
- Ramp 외란 → 초기 0.2 정도의 오차 발생, 시간이 지나면 안정화

---

## 📁 프로젝트 파일 구성
```
/code
    ControlSystemFinal_ChoiChiHyuk.m
/simulink
    week_4.slx
    estimator.slx
    estimator_syme.slx
    estimator_autonomous.slx
    estimator_integrator.slx
    estimator_integrator_syme.slx
```

---

## ✅ 프로젝트 소감
> 직접 코딩 및 시뮬레이션하면서 이론과 실습의 차이를 체감했고, 제어 시스템 설계의 기초와 실제 구현 능력을 한 단계 더 높일 수 있는 좋은 경험이었습니다.

---

## ⭐ 참고 자료
- 강의 교재 및 실습 자료
- [MathWorks 공식 Ackermann 문서](https://www.mathworks.com/help/control/ref/acker.html)
- Ogata, Modern Control Engineering

