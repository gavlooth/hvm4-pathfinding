"""
    KwonSpeedLoss

Kwon (2008) speed loss model for ship weather routing.
Computes percentage speed loss due to added resistance in wind and waves.

Reference: Kwon, Y.J. "Speed loss due to added resistance in wind and waves."
           The Naval Architect, March 2008.
"""
module KwonSpeedLoss

export ShipParams, kwon_speed_loss, kwon_edge_cost, wind_to_beaufort

"""
Ship parameters for the Kwon model.

- `V_design`: design service speed (knots)
- `L_wl`: waterline length (m)
- `displacement`: displacement volume (m³)
- `C_B`: block coefficient (0.55 = fine, 0.85 = full)
- `condition`: :normal, :laden, or :ballast
- `fuel_rate_base`: base fuel consumption (kg/h)
- `fuel_rate_per_kt`: additional fuel per knot of speed (kg/h/kt)
"""
struct ShipParams
    V_design::Float64       # design speed (knots)
    L_wl::Float64           # waterline length (m)
    displacement::Float64   # displacement volume (m³)
    C_B::Float64            # block coefficient
    condition::Symbol       # :normal, :laden, :ballast
    fuel_rate_base::Float64
    fuel_rate_per_kt::Float64
end

# Default: medium containership (C_B=0.65, 200m, 30000 m³, 15 kt)
function ShipParams(;
    V_design=15.0, L_wl=200.0, displacement=30000.0,
    C_B=0.65, condition=:normal,
    fuel_rate_base=50.0, fuel_rate_per_kt=5.0
)
    ShipParams(V_design, L_wl, displacement, C_B, condition,
               fuel_rate_base, fuel_rate_per_kt)
end

"""
    wind_to_beaufort(wind_speed_ms) -> Float64

Convert wind speed (m/s) to continuous Beaufort number.
Inverse of: wind_speed = 0.836 * BN^(3/2)
"""
function wind_to_beaufort(wind_speed_ms::Real)
    wind_speed_ms <= 0.0 && return 0.0
    return (wind_speed_ms / 0.836)^(2/3)
end

"""
    froude_number(V_knots, L_m) -> Float64

Compute Froude number from speed (knots) and waterline length (m).
"""
function froude_number(V_knots::Real, L_m::Real)
    V_ms = V_knots * 0.5144  # knots to m/s
    return V_ms / sqrt(9.81 * L_m)
end

"""
    head_weather_loss(BN, displacement, C_B, condition) -> Float64

Percentage speed loss in head seas (Eq 1a/1b/1c from Kwon 2008).
Returns ΔV/V as a fraction (not percentage).
"""
function head_weather_loss(BN::Real, displacement::Real, C_B::Real, condition::Symbol)
    BN <= 0.0 && return 0.0
    disp_23 = displacement^(2/3)

    if condition == :laden && C_B >= 0.75
        # Eq 1a: laden, C_B = 0.75-0.85
        pct = 0.5 * BN + BN^6.5 / (2.7 * disp_23)
    elseif condition == :ballast
        # Eq 1b: ballast
        pct = 0.7 * BN + BN^6.5 / (2.7 * disp_23)
    else
        # Eq 1c: normal condition (containerships, C_B = 0.55-0.70)
        pct = 0.7 * BN + BN^6.5 / (22.0 * disp_23)
    end

    return pct / 100.0  # convert percentage to fraction
end

"""
    correction_alpha(C_B, Fn, condition) -> Float64

Correction factor α from Table 1 (Kwon 2008).
Interpolates between tabulated C_B values.
"""
function correction_alpha(C_B::Real, Fn::Real, condition::Symbol)
    # Clamp inputs to valid range
    Fn = clamp(Fn, 0.0, 0.45)
    C_B = clamp(C_B, 0.55, 0.85)

    if condition == :ballast
        return _alpha_ballast(C_B, Fn)
    elseif condition == :laden
        return _alpha_laden(C_B, Fn)
    else
        return _alpha_normal(C_B, Fn)
    end
end

# Table 1 coefficients: α = a0 + a1·Fn + a2·Fn²
const _ALPHA_NORMAL = [
    # C_B   a0   a1     a2
    (0.55, 1.7, -1.4,  -7.4),
    (0.60, 2.2, -2.5,  -9.7),
    (0.65, 2.6, -3.7,  -11.6),
    (0.70, 3.1, -5.3,  -12.4),
]

const _ALPHA_LADEN = [
    (0.75, 2.4, -10.6, -9.5),
    (0.80, 2.6, -13.1, -15.1),
    (0.85, 3.1, -18.7, 28.0),
]

const _ALPHA_BALLAST = [
    (0.75, 2.6, -12.5, -13.5),
    (0.80, 3.0, -16.3, -21.6),
    (0.85, 3.4, -20.9, 31.8),
]

function _interp_alpha(table, C_B, Fn)
    # Find bracketing entries and interpolate
    if length(table) == 1
        _, a0, a1, a2 = table[1]
        return a0 + a1 * Fn + a2 * Fn^2
    end

    # Clamp to table range
    cb_lo = table[1][1]
    cb_hi = table[end][1]
    C_B = clamp(C_B, cb_lo, cb_hi)

    # Find bracket
    for i in 1:length(table)-1
        cb0, a0_0, a1_0, a2_0 = table[i]
        cb1, a0_1, a1_1, a2_1 = table[i+1]
        if C_B <= cb1
            t = (cb1 == cb0) ? 0.0 : (C_B - cb0) / (cb1 - cb0)
            a0 = a0_0 + t * (a0_1 - a0_0)
            a1 = a1_0 + t * (a1_1 - a1_0)
            a2 = a2_0 + t * (a2_1 - a2_0)
            return a0 + a1 * Fn + a2 * Fn^2
        end
    end

    # Fallback: last entry
    _, a0, a1, a2 = table[end]
    return a0 + a1 * Fn + a2 * Fn^2
end

_alpha_normal(C_B, Fn) = _interp_alpha(_ALPHA_NORMAL, C_B, Fn)
_alpha_laden(C_B, Fn) = _interp_alpha(_ALPHA_LADEN, C_B, Fn)
_alpha_ballast(C_B, Fn) = _interp_alpha(_ALPHA_BALLAST, C_B, Fn)

"""
    direction_factor(weather_angle_deg, BN) -> Float64

Weather direction reduction factor μ (from Aertssen via Kwon 2008).
`weather_angle_deg`: angle between weather direction and ship heading (0° = head sea).
"""
function direction_factor(weather_angle_deg::Real, BN::Real)
    θ = abs(weather_angle_deg)
    if θ > 180.0
        θ = 360.0 - θ
    end

    if θ <= 30.0
        # Head sea: full effect
        return 1.0
    elseif θ <= 60.0
        # Bow seas
        two_mu = 1.7 - 0.03 * (BN - 4)^2
        return max(clamp(two_mu / 2.0, 0.0, 1.0), 0.0)
    elseif θ <= 150.0
        # Beam seas
        two_mu = 0.9 - 0.06 * (BN - 6)^2
        return max(clamp(two_mu / 2.0, 0.0, 1.0), 0.0)
    else
        # Following seas
        two_mu = 0.4 - 0.03 * (BN - 8)^2
        return max(clamp(two_mu / 2.0, -0.2, 1.0), -0.2)  # can be slightly negative (speed boost)
    end
end

"""
    kwon_speed_loss(ship, wind_speed_ms, wind_dir_deg, heading_deg) -> Float64

Compute fractional speed loss ΔV/V using the full Kwon model.
Returns a value in [0, 1) where 0 = no loss, 0.5 = 50% speed reduction.
"""
function kwon_speed_loss(ship::ShipParams, wind_speed_ms::Real, wind_dir_deg::Real, heading_deg::Real)
    BN = wind_to_beaufort(wind_speed_ms)
    BN <= 0.0 && return 0.0

    Fn = froude_number(ship.V_design, ship.L_wl)
    α = correction_alpha(ship.C_B, Fn, ship.condition)
    loss_head = head_weather_loss(BN, ship.displacement, ship.C_B, ship.condition)

    weather_angle = wind_dir_deg - heading_deg
    μ = direction_factor(weather_angle, BN)

    # Total fractional speed loss (clamp to prevent negative speed)
    return clamp(α * μ * loss_head, 0.0, 0.95)
end

"""
    kwon_edge_cost(ship, dist_m, heading_deg, wind_speed_ms, wind_dir_deg,
                   current_speed_ms, current_dir_deg;
                   mode=:time, alpha=0.5, safety_bn=7.0, safety_penalty=100.0,
                   scale=1000) -> UInt32

Compute integer edge weight for a single edge using the Kwon model.

Returns cost × scale as UInt32, suitable for shortest-path algorithms.
"""
function kwon_edge_cost(
    ship::ShipParams,
    dist_m::Real,
    heading_deg::Real,
    wind_speed_ms::Real,
    wind_dir_deg::Real,
    current_speed_ms::Real,
    current_dir_deg::Real;
    mode::Symbol = :time,
    alpha::Float64 = 0.5,
    safety_bn::Float64 = 7.0,
    safety_penalty::Float64 = 100.0,
    scale::Int = 1000
)
    # Speed loss from wind/waves
    frac_loss = kwon_speed_loss(ship, wind_speed_ms, wind_dir_deg, heading_deg)

    # Effective speed (knots)
    V_eff = ship.V_design * (1.0 - frac_loss)

    # Current contribution (m/s → knots, projected along heading)
    current_angle_rad = deg2rad(current_dir_deg - heading_deg)
    current_along_kt = current_speed_ms * cos(current_angle_rad) * 1.94384
    V_eff += current_along_kt

    # Minimum steerage speed
    V_eff = max(V_eff, 0.5)

    # Time cost
    dist_nm = dist_m / 1852.0
    time_h = dist_nm / V_eff

    # Fuel cost
    fuel_kg = time_h * (ship.fuel_rate_base + ship.fuel_rate_per_kt * V_eff)

    # Combined cost
    cost = if mode == :time
        time_h
    elseif mode == :fuel
        fuel_kg
    elseif mode == :weighted
        alpha * time_h + (1.0 - alpha) * fuel_kg
    elseif mode == :safety
        c = alpha * time_h + (1.0 - alpha) * fuel_kg
        BN = wind_to_beaufort(wind_speed_ms)
        BN > safety_bn ? c + safety_penalty : c
    else
        time_h
    end

    # Scale to integer
    w = round(UInt32, cost * scale)
    return max(w, UInt32(1))
end

end # module
