# ✅ Control System Design Final Term Project
**팀원:**  
- 2020741010 최치혁  
- 2020741050 한승엽  

---

## 📚 1. System Selection
- 모터제어 및 로봇학 실험에서 배운 내용을 바탕으로 DC 모터 시스템을 선정.  
- 교재 내 가장 표준적인 모델을 실습 대상으로 삼음.

---

## 🧮 2. System Equations
\[
\tau = K_m i, \quad V_{emf} = K_b \omega
\]
\[
J \cdot \frac{d\omega}{dt} = -K_f \omega + K_m i
\]
\[
\frac{di}{dt} = -\frac{R}{L} i - \frac{K_b}{L} \omega + \frac{1}{L} V_{app}
\]

---

## 📐 3. State Space Representation
\[
\dot{x} = A x + B u, \quad y = C x + D u
\]
- **A, B, C, D Matrix**
\[
A = \begin{bmatrix}
-\frac{R}{L} & -\frac{K_b}{L} \\
\frac{K_m}{J} & -\frac{K_f}{J}
\end{bmatrix}, 
B = \begin{bmatrix}
\frac{1}{L} \\ 0
\end{bmatrix}, 
C = [0 \quad 1], 
D = [0]
\]
- 실제 사용 값:
    - \( R=2 \Omega, L=0.5H, K_m=K_b=0.015, J=0.02 \, kg \cdot m^2, K_f=0.2 \, N \cdot m \cdot s \)

---

## 🎯 4. Performance Index
- 감쇠비(ζ) = 0.5 이상  
- Rise time = 2 sec  
- Natural frequency = 1 rad/sec  
- Settling time = 9.2 sec  
- Overshoot ≤ 16%

---

## ✅ 5. Controllability Check
- 코드로 controllability matrix 생성 및 rank 확인:
```matlab
controllability_matrix = [B, A*B];
rank_C = rank(controllability_matrix);
