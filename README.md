# Jansen 12-Link Gait Trainer: Kinematic Model

This note explains the MATLAB file `Jansen_Mechanism.m`.
It follows the mechanism topology shown in the supplied papers:

- Shin, Deshpande, and Sulzer, "Design of a Single Degree-of-Freedom, Adaptable Electromechanical Gait Trainer for People With Neurological Injury", JMR 2018.
- Jadav et al., "Kinematic Performance of a Customizable Single Degree-of-Freedom Gait Trainer...", supplied as `sn-article.pdf`.

The later paper gives the experimental link-length table:

| link       |  L1 |  L2 |  L3 |  L4 |   L5 |   L6 |   L7 |   L8 |  L9 | L10 |  L11 |  L12 |
| ---------- | --: | --: | --: | --: | ---: | ---: | ---: | ---: | --: | --: | ---: | ---: |
| nominal cm |  11 |  45 |  36 |  34 | 48.5 | 41.5 | 60.5 | 41.5 |  42 |  43 | 26.5 | 54.5 |

The script uses the paper's example adjustable values:

`L1 = 11.29 cm`, `L4 = 32.93 cm`, `L8 = 41.78 cm`.

With the gait/world frame rotated by `-11.4 deg` relative to the schematic ground-link frame, the script gives approximately:

`x-span = 50.08 cm`, `y-span = 12.74 cm`.

This matches the paper-reported simulated output for the chosen human gait envelope, approximately `[50.10, 12.75] cm`.

## 1. Coordinate Frames

The mechanism is solved first in a mechanism frame `M`.

- `P0 = [0, 0]^T` is the crank ground pivot.
- `P3 = [L4, 0]^T` is the second ground pivot.
- Link `L4` is therefore the fixed ground link.
- Link `L1` is the input crank.
- The input angle is `theta1`, swept from `0` to `2*pi`.

The gait/world frame `G` is obtained by rotating all solved points by:

`alpha = -11.4 deg`.

That rotation does not change the mechanism closure. It only expresses the ankle path in the same horizontal/vertical gait axes used for stride length and step height.

## 2. Link Direction Convention

Define:

`e(theta) = [cos(theta), sin(theta)]^T`.

The script uses link angles `theta1 ... theta12` with these vector directions:

- `L1`: `P0 -> P1`
- `L2`: `P1 -> P2`
- `L3`: `P3 -> P2`
- `L4`: `P0 -> P3`, fixed, so `theta4 = 0`
- `L5`: `P2 -> P4`
- `L6`: `P3 -> P4`
- `L7`: `P1 -> P5`
- `L8`: `P3 -> P5`
- `L9`: `P5 -> P6`
- `L10`: `P4 -> P6`
- `L11`: `P5 -> PE`
- `L12`: `PE -> P6`

The end-effector/ankle point is `PE`, the joint between `L11` and `L12`.

The unknown angle vector at each crank angle is:

`q = [theta2, theta3, theta5, theta6, theta7, theta8, theta9, theta10, theta11, theta12]^T`.

`theta1` is prescribed. `theta4` is fixed.

## 3. Closed Vector Loops

The mechanism has five closure equations, each with x and y components, so there are 10 scalar equations for 10 unknown angles.

### Upper four-bar: L1-L2-L3-L4

Path: `P0 -> P1 -> P2 -> P3 -> P0`

`L1*e1 + L2*e2 - L3*e3 - L4*e4 = 0`

### Lower four-bar: L1-L7-L8-L4

Path: `P0 -> P1 -> P5 -> P3 -> P0`

`L1*e1 + L7*e7 - L8*e8 - L4*e4 = 0`

### Coupler triangle: L3-L5-L6

Path: `P3 -> P2 -> P4 -> P3`

`L3*e3 + L5*e5 - L6*e6 = 0`

### Parallelogram-like loop: L6-L8-L9-L10

Path: `P3 -> P5 -> P6 -> P4 -> P3`

`L8*e8 + L9*e9 - L6*e6 - L10*e10 = 0`

The paper calls this a parallelogram mechanism. The optimized/experimental lengths are close but not exactly equal in opposite pairs, so the code treats it as a general four-bar closure.

### Foot triangle: L9-L11-L12

Path: `P5 -> PE -> P6 -> P5`

`L11*e11 + L12*e12 - L9*e9 = 0`

Together:

`F(q, theta1, L) = 0`.

This is implemented in `loopEquations`.

## 4. Forward Kinematics

Once the angles are solved, joint positions are computed as:

```matlab
P0 = [0; 0];
P3 = [L4; 0];
P1 = P0 + L1*e(theta1);
P2 = P1 + L2*e(theta2);
P5 = P1 + L7*e(theta7);
P4 = P2 + L5*e(theta5);
P6 = P5 + L9*e(theta9);
PE = P5 + L11*e(theta11);
```

The equal alternative definitions,

```matlab
P2 = P3 + L3*e(theta3);
P5 = P3 + L8*e(theta8);
P4 = P3 + L6*e(theta6);
P6 = P4 + L10*e(theta10);
PE = P6 - L12*e(theta12);
```

are enforced by the vector loops.

## 5. Numerical Solving

For each `theta1`:

1. A geometric circle-intersection assembly is used only as a branch seed.
2. The nonlinear loop equations are solved.
3. The previous solution is used as the next initial guess, which preserves the physical assembly mode through a full revolution.

The script uses MATLAB `fsolve` if Optimization Toolbox is installed.
This machine did not have `fsolve`, so the file also contains a damped Newton/Gauss-Newton solver with finite-difference Jacobians. It solves the same equations and reached residuals near `1e-14 cm` during verification.

## 6. Plots Produced

The script generates:

- A paper-style composite figure similar to Fig. 4: mechanism sketch, endpoint markers, and x/y gait-cycle curves.
- A paper-style 3x3 validation grid with reference-vs-simulation endpoint paths and RMSE labels.
- A live mechanism animation when the script is run in the MATLAB desktop UI.
- End-effector trajectory `x` versus `y`.
- `x` and `y` versus gait cycle percent.
- End-effector velocity curves for constant crank speed.
- Closed-loop residual validation.
- Mechanism snapshots.
- Sensitivity of stride/height to adjustable links `L1`, `L4`, and `L8`.
- Least-squares span-to-link mapping demo.

## 7. Effect of Adjustable Links

`L1` is the crank radius. Increasing it generally increases excitation of both four-bars, changing both stride length and vertical lift.

`L4` is the ground pivot separation. Changing it shifts the base geometry of both four-bars, strongly affecting horizontal span and the timing/shape of the swing arc.

`L8` belongs to the lower four-bar and the parallelogram-like loop. It strongly changes the lower joint `P5`, which then moves the foot triangle, so it is important for step height and endpoint-loop shape.

This is why the papers focus on `L4` and `L8`, and the later paper also makes `L1` adjustable.

## 8. Least-Squares Span Mapping

The paper describes an empirical map from desired gait spans to adjustable link lengths.

Let:

- `Lambda in R^(3xn)` store sampled `[L1; L4; L8]`.
- `Sigma in R^(2xn)` store simulated `[xspan; yspan]`.
- `Psi in R^(3x2)` map spans to links.

The least-squares solution is:

`Psi = Lambda * pinv(Sigma)`.

For a new desired span:

`[L1; L4; L8] ~= Psi * [xspan; yspan]`.

The script implements this in `runLeastSquaresSpanMap`.

## 9. Important Accuracy Note

The PDFs do not include the raw human ankle marker data or the authors' MATLAB code.
Therefore the included blue "reference" curve is only a smooth gait-like envelope for visual comparison. The actual reproduced research-level part is the closed-chain mechanism kinematics and the output envelope. With the supplied example adjustable lengths, the simulated stride/height spans match the paper's reported values.
