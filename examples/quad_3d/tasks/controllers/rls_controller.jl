
function init_rls_controller()

    # Load parameters from a YAML file
    params = YAML.load(open("rls_controller_params.yaml"))
    m = params["m"]
    g = params["g"]
    f = params["f"]
    T = params["T"]
    W = params["W_RLS"]
    is_sim = params["is_sim"]
    gamma = params["gamma_RLS"]
    mode = params["mode"]
    target = params["target"]
    delta_t = 1.0 / f
    filtering = params["filtering"]
    process_noise_var = params["process_noise_var"]
    measurement_noise_var = params["measurement_noise_var"]

    # initialize target dynamics estimation
    self.idx_of_interest = [0, 1, 2, 3, 4, 5] # the indices of target states that can be measured and learned. In our case is [x,y,z,vx,vy,vz]
    self.n_of_interest = length(self.idx_of_interest)
    S_target = Matrix{Float64}(I, self.n_of_interest, self.n_of_interest) # r_t=[x, y, z, vx, vy, vz]'
    S_target_aug = copy(S_target)

    self.S_target_aug_all = [deepcopy([S_target_aug for _ in 1:self.W]) for _ in 1:self.W]

    P = 1e-5 * Matrix{Float64}(I, self.n_of_interest, self.n_of_interest) # P_t = \gamma*P_{t-1} + r_{t-1}r_{t-1}'
    self.P_all = [deepcopy([copy(P) for _ in 1:self.W]) for _ in 1:self.W]

    self.pred_log = []
    self.error_log = []

    # precompute LQR gains
    self.compute_A_tilde_power()
    self.solve_M_optimal_all()

    # initialize target position in case it is not detected by the camera.
    self.desired_pos = [0, 0.65, 0.3]

    self.disturbances_predicted = []
end

function predict_future_targets(self)
    """
    Predict the future targets and compute the corresponding disturbances up to horizon W.
    Note that only the indices of interest are predicted, other indices are set to default values.
    """
    timesteps = length(self.target_state_log)
    pred_target_state_all = []
    curr_target_state = self.target_state_log[end][self.idx_of_interest]
    last_pred_target_state_full = self.target_state_log[end]
    self.disturbances_predicted = []
    for k in 1:self.W
        learner_idx = (timesteps - 1) % k
        S_target_k_step = self.S_target_aug_all[k][learner_idx][1:self.n_of_interest, 1:self.n_of_interest]
        pred_target_state = S_target_k_step * curr_target_state
        push!(pred_target_state_all, pred_target_state)
        pred_target_state_full = zeros(9, 1)
        pred_target_state_full[self.idx_of_interest] = pred_target_state
        disturbance = self.A * last_pred_target_state_full - pred_target_state_full
        push!(self.disturbances_predicted, disturbance)
        last_pred_target_state_full = pred_target_state_full
    end
    push!(self.pred_log, pred_target_state_all)
end

function compute_A_tilde_power(self)
    """
    Precompute (A-BK)^i, i=0,...,H
    """
    self.A_tilde_power = [I(9)]
    for _ in 1:self.W-1
        new = (self.A - self.B * self.K_star) * self.A_tilde_power[end]
        push!(self.A_tilde_power, new)
    end
end


function solve_M_optimal_all(self)
    """
    compute optimal disturbance-feedback gains
    """
    self.M_optimal_all = []
    inv = inv(self.R + self.B' * self.P_star * self.B)
    for i in 1:self.W
        M_optimal_i = inv * self.B' * self.A_tilde_power[i]' * self.P_star
        push!(self.M_optimal_all, M_optimal_i)
    end
end

function projection(self, M, bound)
    """
    Project M to the closest(Frobenius norm) matrix whose l2 norm is bounded.
    """
    U, s, Vh = svd(M)
    for i in 1:4
        s[i] = min(s[i], bound)
    end
    M_projected = U * Diagonal(s) * Vh
    return M_projected
end
