# Linear Kalman Filter Equations

## 1. System Model (State Space Representation)

- $x_k$ = State vector at time $k$
- $F_k$ = State transition matrix (models the dynamics of the system)
- $u_k$ = Control input (optional, if no control input, this term can be ignored)
- $G_k$ = Control input matrix (how the control input affects the state)
- $w_k$ = Process noise (assumed to be zero-mean, Gaussian with covariance $Q_k$)

The system model is defined by:
$$
x_k = F_k x_{k-1} + G_k u_{k-1} + w_k
$$
Where:
- $w_k \sim N(0, Q_k)$

## 2. Measurement Model

- $z_k$ = Measurement vector at time $k$
- $H_k$ = Observation matrix (how the state relates to the measurements)
- $v_k$ = Measurement noise (assumed to be zero-mean, Gaussian with covariance $R_k$)

The measurement model is given by:
$$
z_k = H_k x_k + v_k
$$
Where:
- $v_k \sim N(0, R_k)$

## 3. Predict Step (Time Update)

This step predicts the next state based on the current state estimate and the process model:

- **State prediction:**
  $$
  \hat{x}_k^- = F_k \hat{x}_{k-1} + G_k u_{k-1}
  $$
  Where $\hat{x}_k^-$ is the predicted state estimate.

- **Covariance prediction:**
  $$
  P_k^- = F_k P_{k-1} F_k^T + Q_k
  $$
  Where $P_k^-$ is the predicted covariance estimate, and $P_{k-1}$ is the previous state covariance.

## 4. Update Step (Measurement Update)

Once a measurement is received, the Kalman filter corrects its prediction:

- **Kalman Gain:**
  $$
  K_k = P_k^- H_k^T \left( H_k P_k^- H_k^T + R_k \right)^{-1}
  $$
  The Kalman gain $K_k$ determines how much of the new measurement to incorporate into the state estimate.

- **State update:**
  $$
  \hat{x}_k = \hat{x}_k^- + K_k \left( z_k - H_k \hat{x}_k^- \right)
  $$
  This corrects the predicted state $\hat{x}_k^-$ based on the measurement residual $z_k - H_k \hat{x}_k^-$.

- **Covariance update:**
  $$
  P_k = \left( I - K_k H_k \right) P_k^-
  $$
  This updates the error covariance matrix based on the Kalman gain and observation matrix.

## 5. Process and Measurement Noise Assumptions

- **Process Noise $w_k$**:  
  $w_k$ is typically modeled as Gaussian noise with zero mean and covariance $Q_k$:
  $$
  w_k \sim N(0, Q_k)
  $$

- **Measurement Noise $v_k$**:  
  $v_k$ is also modeled as Gaussian noise with zero mean and covariance $R_k$:
  $$
  v_k \sim N(0, R_k)
  $$

## 6. Covariance Matrices
- $P_k$: Estimate covariance matrix (how uncertain we are about the state estimate)
- $Q_k$: Process noise covariance matrix (describes how much uncertainty is added in the process model)
- $R_k$: Measurement noise covariance matrix (describes how uncertain the sensor measurements are)
