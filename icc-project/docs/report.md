# [학번-이름] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄
**제출일**: 2026-06-23
**팀**: 개인

---

## 1. 설계 개요 (1 페이지)

1-2 문단으로:
이 프로젝트의 목표는 차량의 회앙향, 종방향 및 수직방향 거동을 통합해서 제어하는 ICC 시스템을 설계하는 것이다. 차량은 고속 회피 기동, 제동, 정상 선회 등 다양한 상황에서 안정성을 유지해야 하며, 특히 차량의 Slip Angle 증가, Yaw Rate 과도 응답, 전복 위험 등을 효과적으로 억제해야 한다. 이를 위해 Active Front Steering, Electronic Stability Control, Anti-lock Braking System, Continuous Damping Control을 하나의 통합 제어 구조로 구현하였다. PID 제어, Gain Scheduling, Skyhook Damping Control 개념을 적용하였다. 횡방향 제어에는 PID 기반 Yaw Rate Tracking 제어기를 사용하였으며, 차량의 Slip Angle이 임계값을 초과하는 경우 ESC를 통해 추가적인 Yaw Moment를 발생시켰다. 종방향 제어에는 PI 기반 속도 추종 제어기를 사용하였으며, Wheel Slip 조건을 이용한 ABS Logic을 구현하였다. 수직방향 제어에는 Skyhook 기반 Semi-active Suspension Control을 적용하여 차체 진동을 감소시키고 승차감을 향상시켰다. 마지막으로 Coordinator를 통해 상위 제어기의 명령을 실제 조향각, 브레이크 토크 및 감쇠계수로 변환하였다.

각 제어기 한 줄 요약:
- **ctrl_lateral**: PID 기반 Yaw Rate Tracking + ESC 기반 Slip Angle Limiter
- **ctrl_longitudinal**: PI 기반 속도 제어 + ABS Logic
- **ctrl_vertical**: Skyhook 기반 Semi-active Damping Control
- **ctrl_coordinator**: <yaw moment → brake 분배 방식> Yaw Moment를 Differential Brake Torque로 변환하는 Actuator Allocation

---

## 2. 수학적 모델링 (1-2 페이지)

### 2.1 사용한 plant 단순화
어떤 모델을 제어 설계에 사용했는가? (bicycle? 3DOF?) 학생은 14DOF plant 위에 검증하지만, **제어기 설계** 자체는 보통 더 단순한 모델 위에서 한다.
최종 검증은 14-DOF Vehicle Model에서 수행되었다. 그러나 제어기 설계 단계에서는 보다 단순한 Bicycle Model을 사용하였다. Bicycle Model은 좌우 바퀴를 각각 하나의 가상 바퀴로 통합한 모델이며 차량의 횡방향 거동과 Yaw Motion을 비교적 정확하게 표현할 수 있다. 

### 2.2 State-space 표현
$$\dot{x} = Ax + Bu, \quad y = Cx + Du$$

상태 변수, 입력, 출력 정의 + A, B 행렬 표현. Bicycle Model 사용 시:
$$x = [v_y, r]^T, \quad u = \delta$$
$$\dot{v}_y = -(C_f + C_r)/(mV_x)\,v_y + ((l_r C_r - l_f C_f)/(mV_x) - V_x)\,r + C_f/m\,\delta$$
$$\dot{r} = (l_r C_r - l_f C_f)/(I_z V_x)\,v_y - (l_f^2 C_f + l_r^2 C_r)/(I_z V_x)\,r + l_f C_f/I_z\,\delta$$



### 2.3 가정 + 한계
- 일정 종속도 (제어 설계 시 분리)
- 선형 타이어 (소슬립 영역)
- 그 외 본인이 사용한 가정
제어 설계 단계에서 일정 종속도를 가정하였고 타이어는 선형 영역에서 동작한다고 가정하였다. 차량의 롤 및 피치 운동은 Bicycle Model 설계 단계에서 무시하였다 타이어 하중 이동 효과는 고려하지 않았다. 

---

## 3. 제어기 설계 (3-4 페이지)

### 3.1 ctrl_lateral — AFS + ESC

**설계 목표**:
- yaw rate 추종 (settling < 0.8s, overshoot < 10%)
- |β| > 3° 시 ESC 개입

**선택 기법**: PID Controller + Gain Scheduling + ESC 기반 Slip Angle Limiter
구현이 간단하고 다양한 주행 조건에서 안정적인 성능을 확보할 수 있는 PID 제어기를 사용하였다. Yaw Rate 오차를 이용하여 추가 조향각(AFS)을 생성하고, Slip Angle이 임계값을 초과하는 경우 ESC를 통해 추가적인 Yaw Moment를 발생시키도록 설계하였다.

**Gain 계산 과정**:
정확한 차량 파라미터 기반의 LQR 설계 대신 PID 제어기를 사용하였다. 초기 게인은 강의에서 학습한 PID 제어 구조를 기반으로 설정하였으며, 이후 시뮬레이션을 반복 수행하면서 경험적 튜닝(Empirical Tuning)을 통해 최종 게인을 결정하였다.

Yaw Rate 오차는 다음과 같이 정의하였다.

$$e_r=r_{ref}-r$$

추가 조향각은 다음 식으로 계산하였다.

$$\delta_{AFS}=K_p e_r+K_i\int e_rdt+K_d\frac{de_r}{dt}$$

또한 차량 속도에 따른 응답 특성 변화를 고려하여 Gain Scheduling을 적용하였다.

$$GainScale=min\left(\frac{V_x}{30},1.5\right)$$

최종 제어 입력은 다음과 같이 계산하였다.

$$\delta_{cmd}=GainScale\cdot\delta_{AFS}$$

Slip Angle이 임계값을 초과하는 경우 ESC를 동작시켰다.

$$|\beta|>\beta_{th}$$ 일 때

$$M_z=-K_{\beta},sign(\beta),(|\beta|-\beta_{th})$$ 의 형태로 Yaw Moment를 생성하였다.

**최종 게인 + 정당화**:
```matlab
CTRL.LAT.Kp = 1.0; CTRL.LAT.Ki = 0.1; CTRL.LAT.Kd = 0.05; beta_th = deg2rad(7); yawMomentGain = 1000; 
```

### 3.2 ctrl_longitudinal — 속도 + ABS
**설계 목표**: 목표 속도 추종, 과도한 Wheel Slip 방지, 제동 안정성 향상

**선택 기법**:PI Controller + Anti-Windup + ABS Logic + Jerk Limiter
종방향 속도 오차를 이용한 PI 제어기를 사용하였다. 속도 추종 성능과 계산 효율을 동시에 확보할 수 있으며 구현이 간단하다는 장점이 있다. 또한 급격한 제동 명령 변화로 인한 불안정성을 방지하기 위해 Jerk Limiter를 적용하였다. 

**Gain 계산 과정**:
속도 오차는 다음과 같이 정의하였다.

$$e_v=v_{ref}-v_x$$

종방향 힘 명령은 PI 제어기를 이용하여 계산하였다.

$$F_x=K_p e_v+K_i\int e_vdt$$

적분기의 발산을 방지하기 위하여 Anti-Windup을 적용하였다.

$$I=\max(-I_{max},\min(I,I_{max}))$$

또한 제어 입력 변화율을 제한하기 위해 Jerk Limiter를 적용하였다.

$$\Delta F_x \le J_{max} \cdot m \cdot dt$$

ABS 기능은 Wheel Slip이 일정 수준 이상 증가할 경우 제동력을 감소시키는 방식으로 구현하였다.

$$|\kappa|>0.12$$ 또는 $$a_x<-4.0m/s^2$$
조건을 만족할 경우

$$F_x=0.6F_x$$ 로 감소시켜 Wheel Lock을 방지하도록 하였다.

**최종 게인 + 정당화**:
CTRL.LON.Kp = 0.5;
CTRL.LON.Ki = 0.05;
CTRL.LON.intMax = 2000;

최종 게인은 여러 시뮬레이션을 반복 수행하며 경험적으로 조정하였다.

### 3.3 ctrl_vertical — CDC (있다면)
**설계 목표**: 승차감 향상, 차체 진동 감소, Roll 및 Pitch 억제

**선택 기법**:Skyhook Damping Control
Semi-active Suspension의 대표적인 제어 방식인 Skyhook Control을 사용하였다. Skyhook 제어는 차체 속도를 감소시키는 방향으로 감쇠력을 생성하여 차체 진동을 효과적으로 억제할 수 있다.

**Gain 계산 과정**:
Skyhook 제어는 Sprung Mass와 Unsprung Mass의 상대 운동을 이용하여 감쇠계수를 조절한다.
다음 조건을 만족하는 경우

$$z_s'(z_s'-z_u')>0$$

감쇠계수를 최대값으로 설정하였다.

$$c=c_{max}$$

반대로 조건을 만족하지 않는 경우에는 최소 감쇠계수를 사용하였다.

$$c=c_{min}$$

이를 통해 차체 운동을 억제하면서도 과도한 승차감 저하를 방지하였다.


**최종 게인 + 정당화**:
CTRL.VER.cMin = 500;
CTRL.VER.cMax = 5000;
CTRL.VER.skyGain = 2500;
Skyhook 제어는 특히 A7 및 D1 시나리오에서 Side Slip과 LTR 감소에 기여하였다.

### 3.4 ctrl_coordinator — Actuator Allocation
Rule-Based Actuator Allocation 방식을 사용하였다. 상위 제어기에서 생성한 종방향 제동력(Fx), ESC Yaw Moment(Mz), Skyhook Damping Command를 실제 차량 액추에이터 명령으로 변환하였다.

종방향 제어기에서 생성된 제동력은 전륜과 후륜에 60:40 비율로 분배하였다.
yaw moment → 4-wheel brake 차동 분배:
$$T_f = 0.6F_xr_w,\quad T_r = 0.4F_xr_w$$

전륜과 후륜의 제동 토크는 각각 좌우 바퀴에 동일하게 분배하였다.

$$T_{FL}=T_{FR}=T_f/2$$
$$T_{RL}=T_{RR}=T_r/2$$
ESC에서 생성된 Yaw Moment는 좌우 차동 제동으로 변환하였다.

$$\Delta T_f=\frac{0.6M_z}{track_f},\quad \Delta T_r=\frac{0.4M_z}{track_r}$$

최종 제동 토크는 다음과 같이 계산하였다.
$$T_{FL}=T_{FL}+\Delta T_f$$
$$T_{FR}=T_{FR}-\Delta T_f$$
$$T_{RL}=T_{RL}+\Delta T_r$$
$$T_{RR}=T_{RR}-\Delta T_r$$

마지막으로 액추에이터 한계를 고려하여 브레이크 토크를 포화(Saturation) 처리하였다.
$$0\le T_{brake}\le T_{max}$$

수직방향 제어기에서 생성된 감쇠계수 명령은 별도의 추가 가공 없이 서스펜션 액추에이터에 직접 전달하였다.

이와 같은 구조를 통해 횡방향 안정성 제어(ESC), 종방향 제동 제어(ABS/Brake Control), 수직방향 승차감 제어(CDC)를 통합적으로 수행하였다.

---

## 4. 시뮬레이션 결과 (2-3 페이지)

### 4.1 P1 시나리오 benchmark — 베이스라인 vs 본인 설계

========================= KPI Comparison =========================
scn    | KPI                    |        OFF |         ON |   delta%
--------------------------------------------------------------------
A1     | sideSlipMax            |     3.0154 |     2.6616 |   -11.7%
A1     | LTR_max                |     0.8635 |     0.7508 |   -13.0%
A1     | tireUtilizationMax     |     0.9994 |     1.0000 |    +0.1%
A1     | lateralDevMax          |     1.8270 |     2.1897 |   +19.9%
A1     | yawRateOvershoot       | 2310049.1737 | 178449.4837 |   -92.3%
--------------------------------------------------------------------
A3     | sideSlipMax            |     1.1138 |     0.7558 |   -32.1%
A3     | LTR_max                |     0.4157 |     0.3191 |   -23.2%
A3     | tireUtilizationMax     |     0.6531 |     1.0000 |   +53.1%
A3     | yawRateOvershoot       |     2.6997 |     1.8521 |   -31.4%
--------------------------------------------------------------------
A4     | sideSlipMax            |     1.1839 |     1.1810 |    -0.2%
A4     | LTR_max                |     0.0258 |     0.0257 |    -0.3%
A4     | tireUtilizationMax     |     0.6680 |     0.7215 |    +8.0%
--------------------------------------------------------------------
A7     | sideSlipMax            |    30.4776 |     1.4594 |   -95.2%
A7     | LTR_max                |     0.6808 |     0.2387 |   -64.9%
A7     | tireUtilizationMax     |     1.0467 |     0.9994 |    -4.5%
--------------------------------------------------------------------
B1     | sideSlipMax            |     0.0000 |     0.0000 |   n/a  
B1     | LTR_max                |     0.0000 |     0.0000 |   n/a  
B1     | stoppingDistance       |    72.2992 |    72.2992 |    +0.0%
B1     | jerkMax                |  1942.8793 |  1942.8793 |    +0.0%
--------------------------------------------------------------------
D1     | sideSlipMax            |     4.9057 |     3.6296 |   -26.0%
D1     | LTR_max                |     0.8635 |     0.7508 |   -13.1%
D1     | tireUtilizationMax     |     1.0988 |     1.0487 |    -4.6%
D1     | lateralDevMax          |     1.8270 |     2.1897 |   +19.9%
D1     | yawRateOvershoot       | 8601467.6431 | 83933.5653 |   -99.0%
D1     | jerkMax                |  1637.2824 |  2316.8056 |   +41.5%
--------------------------------------------------------------------
Auto-graded total: 52.24 / 70.00 (보고서 평가 추가 시 max 100)

(`run('scripts/run_icc_benchmark.m')` 출력 + `run('scripts/grade.m')` 점수를 같이 표기)

### 4.2 핵심 plot — A1 DLC

![A1 trajectory comparison](figures/a1_trajectory.png)
*Figure 4.1 — A1 ISO 3888-1 DLC, 차량 trajectory (off vs on) vs reference path.*

![A1 yaw rate](figures/a1_yawrate.png)
*Figure 4.2 — A1 yaw rate 응답: reference (driver bicycle model), off (controller off), on (본인 설계).*

(plot 생성 예시:
```matlab
[r_off, k_off] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on,  k_on ] = run_icc_scenario('A1','14dof','Controller','on', 'SavePlot',false);
figure; plot(r_off.x_pos, r_off.y_pos, 'r--', r_on.x_pos, r_on.y_pos, 'b-', ...
             r_off.scenario.refPath(:,1), r_off.scenario.refPath(:,2), 'k:');
xlabel('x [m]'); ylabel('y [m]'); legend('off','on','ref'); axis equal;
saveas(gcf, 'docs/figures/a1_trajectory.png');
```)

### 4.3 한 시나리오 deep dive — A7 Brake-in-Turn
 A7 Brake-in-Turn 시나리오는 차량이 선회 중 제동을 수행하는 상황을 가정한 시험으로, 차량 동역학 관점에서 가장 불안정한 상황 중 하나이다. 차량이 선회하면서 동시에 제동을 수행할 경우 타이어가 횡력과 종력을 동시에 발생시켜야 하므로 마찰원(Friction Circle)의 한계에 빠르게 도달하게 된다. 이 과정에서 차량은 과도한 Slip Angle 증가, Yaw Rate 불안정, 심한 경우 스핀아웃(Spin-Out) 현상을 보일 수 있다. 따라서 A7 시나리오는 ESC(Electronic Stability Control)의 효과를 검증하기 위한 대표적인 시험으로 활용된다.
 ICC 제어기는 AFS(Active Front Steering), ESC(Electronic Stability Control), ABS Logic 및 CDC(Skyhook Damping Control)를 통합적으로 적용하였다. 특히 A7 시나리오에서는 ESC가 생성하는 추가 Yaw Moment가 차량 자세 안정화에 핵심적인 역할을 수행하였다.

 Benchmark 결과를 비교하면 Controller OFF 상태에서는 최대 Side Slip Angle이 30.48°까지 증가하였다. 이는 차량 후륜이 크게 미끄러지며 차량 자세가 불안정해진 상태를 의미한다. 반면 Controller ON 상태에서는 최대 Side Slip Angle이 1.46°로 감소하였다. 이는 약 95.2% 감소한 결과로, 제안한 제어기가 차량의 횡방향 안정성을 효과적으로 향상시켰음을 보여준다.

 또한 Lateral Load Transfer Ratio(LTR)는 0.6808에서 0.2387로 감소하였다. 이는 약 64.9% 감소한 결과이며, 차량의 롤 거동이 완화되고 전복 위험성이 감소하였음을 의미한다. Brake-in-Turn 상황에서 LTR 감소는 차량의 안정성을 판단하는 중요한 지표이며, 본 제어기가 차량 하중 이동을 효과적으로 억제하였음을 확인할 수 있었다.

 A7 시나리오에서 가장 큰 성능 향상은 ESC 모듈에 의해 발생하였다. 차량의 Slip Angle이 설정된 임계값을 초과하면 ESC는 Slip Angle의 방향과 반대 방향의 Yaw Moment를 생성하도록 설계하였다. 생성된 Yaw Moment는 Coordinator에서 좌우 차동 제동 토크로 변환되며, 차량이 과도하게 회전하려는 경향을 억제한다.
 
 AFS 역시 성능 향상에 기여하였다. PID 기반 Yaw Rate Tracking Controller는 목표 Yaw Rate와 실제 Yaw Rate 사이의 오차를 줄이기 위해 추가 조향각을 생성하였다. 이를 통해 차량은 급격한 선회 상황에서도 보다 안정적으로 목표 경로를 추종할 수 있었다.

 수직방향 제어기인 Skyhook CDC 또한 간접적으로 차량 안정성 향상에 기여하였다. Skyhook 제어는 차체의 진동을 감소시키고 롤 운동을 억제하여 타이어의 접지력을 유지하는 역할을 수행하였다. 그 결과 선회와 제동이 동시에 발생하는 상황에서도 차량이 보다 안정적인 거동을 보일 수 있었다.

 종합적으로 A7 Brake-in-Turn 시나리오는 본 프로젝트에서 가장 큰 성능 향상을 보인 시험 항목이었다. Side Slip Angle은 95.2%, LTR은 64.9% 감소하였으며, 이는 설계한 ICC 제어기가 차량의 횡방향 안정성 향상에 효과적으로 동작하였음을 보여준다. 특히 ESC 기반 Yaw Moment 제어와 AFS 기반 Yaw Rate Tracking의 결합이 성능 개선의 핵심 요인으로 판단된다.



## 5. 분석 + 한계 (1-2 페이지)

### 5.1 가장 성공적이었던 시나리오
가장 큰 성능 향상을 보인 시나리오는 A7 Brake-in-Turn 시나리오였다. A7은 차량이 선회와 제동을 동시에 수행하는 매우 가혹한 조건으로, 차량 자세 안정성이 크게 저하될 수 있는 상황이다.

Benchmark 결과에 따르면 Controller OFF 상태에서는 최대 Side Slip Angle이 30.48°까지 증가하였다. 이는 차량이 심한 오버스티어 상태에 진입하여 스핀아웃에 가까운 거동을 보였음을 의미한다. 반면 제안한 ICC 제어기를 적용한 경우 최대 Side Slip Angle은 1.46°로 감소하였다. 이는 약 95.2% 감소한 결과이며, 전체 시나리오 중 가장 큰 개선 효과를 보였다.

또한 LTR(Lateral Load Transfer Ratio)은 0.6808에서 0.2387로 감소하여 약 64.9%의 개선 효과를 나타냈다. 이는 차량의 롤 거동이 감소하고 전복 위험성이 낮아졌음을 의미한다.

이와 같은 성능 향상의 주요 원인은 ESC 기반 Yaw Moment 제어와 AFS 기반 Yaw Rate Tracking의 결합 효과로 판단된다. 차량의 Slip Angle이 증가하면 ESC가 반대 방향의 Yaw Moment를 생성하여 차량 자세를 안정화하고, PID 기반 AFS가 목표 Yaw Rate를 추종하도록 보조 조향각을 생성함으로써 차량의 횡방향 안정성을 유지하였다. 또한 Skyhook 기반 CDC가 차체 진동을 감소시켜 타이어 접지력을 유지하는 데 기여하였다.

결과적으로 A7 시나리오는 본 프로젝트에서 설계한 통합 섀시 제어기의 효과를 가장 잘 보여주는 시험 항목이었다.

### 5.2 가장 부족했던 시나리오

가장 부족했던 시나리오는 B1 Straight Braking 시나리오였다.

Benchmark 결과를 보면 Controller OFF와 Controller ON 모두 Stopping Distance가 72.30 m로 동일하게 나타났으며, 개선 효과가 거의 관찰되지 않았다. 또한 Grade 결과에서도 Stopping Distance 항목과 ABS Slip RMS 항목이 모두 목표값을 만족하지 못하였다.

원인으로는 종방향 제어기에서 구현한 ABS Logic이 충분히 적극적으로 동작하지 못했기 때문으로 판단된다. 현재 설계에서는 Wheel Slip이 일정 임계값을 초과할 경우 제동력을 감소시키는 비교적 단순한 Bang-Bang 방식의 ABS 구조를 사용하였다. 그러나 실제 차량의 ABS는 Wheel Slip을 지속적으로 추정하며 최적 Slip Ratio 근처를 유지하도록 제어한다.

또한 본 프로젝트의 종방향 제어기는 PI 기반 속도 제어에 초점을 두고 설계되었으며, 제동 거리 최소화를 위한 별도의 최적 제동 전략은 적용하지 않았다. 그 결과 횡방향 안정성은 향상되었지만 B1 시나리오에서 요구하는 제동 성능 향상은 달성하지 못하였다.

A4 정상 선회 시나리오에서도 Understeer Gradient는 목표를 만족하였지만 추가적인 개선은 제한적이었다. 이는 현재 설계가 Slip Angle 억제와 Yaw Stability 확보에 중점을 두었기 때문으로 판단된다.

### 5.3 만약 더 시간이 있었다면

추가적인 시간이 있었더라면 ABS 제어기를 개선하고 싶다. 현재 구현 한 Bang-Bang 방식 대신 Slip Ratio Tracking 기반의 Closed-Loop ABS를 설계한다면 B1 시나리오의 제동거리와 ABS Slip RMS 성능을 크게 향상시킬 수 있을 것으로 예상된다.

Coordinator를 개선하여 보다 정교한 Brake Torque Allocation을 구현하고 싶다. 현재는 Rule-Based 방식으로 토크를 분배하지만, Weighted Least Squares(WLS) 기반 Allocation을 적용하면 여러 액추에이터를 보다 효율적으로 활용할 수 있을 것으로 예상된다.



## 6. 참고문헌

챗지피티

## 부록 A — 사용한 AI 도구

(student_info.m 의 ai_usage 항목과 일치하게)

ChatGPT는 제어기 설계 아이디어 정리, MATLAB 코드 디버깅, PID 제어기 구조 검토, 보고서 작성 보조 및 결과 해석에 사용하였다. 특히 ctrl_lateral, ctrl_longitudinal, ctrl_vertical, ctrl_coordinator 함수의 기본 구조 설계와 MATLAB 문법 오류 수정 과정에서 도움을 받았다.
---

## 부록 B — 본인 sim_params.m 변경사항
없음
