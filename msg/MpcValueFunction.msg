# Local quadratic value and action-value approximation associated with one
# SolverTrajectory interval. All matrices use row-major storage and the same
# state-tangent convention as State.dx and Control.gain.

# True when the action-value fields and active contacts come from a supported
# positive-duration flow action. This is the legacy action validity flag.
bool valid

# True when value_constant/value_gradient/value_hessian are available. The
# value can remain valid at an impulse or terminal-aligned node where no flow
# action value exists.
bool value_valid

# Tangent-state and control dimensions.
uint32 ndx
uint32 nu

# V(dx) = constant + value_gradient' dx
#       + 0.5 dx' value_hessian dx
float64 value_constant
float64[] value_gradient
float64[] value_hessian

# Exact value approximation at the endpoint of this interval.  Keeping the
# endpoint value with its interval ensures that the final executable interval
# of a published horizon still has the Caracal Vx/Vxx needed by a short MPC.
bool endpoint_value_valid
float64 endpoint_value_constant
float64[] endpoint_value_gradient
float64[] endpoint_value_hessian

# Local quadratic approximation of the integrated Caracal running cost at the
# beginning of this interval:
# l(dx,du) = constant + Lx' dx + Lu' du
#          + 0.5 dx' Lxx dx + dx' Lxu du + 0.5 du' Luu du.
# These are running-cost L derivatives, not action-value Q derivatives.
bool running_cost_valid
float64 running_cost_constant
float64[] running_state_gradient
float64[] running_control_gradient
float64[] running_state_hessian
float64[] running_state_control_hessian
float64[] running_control_hessian

# Q(dx,du) = constant + [action_state_gradient; action_control_gradient]'
#                         [dx; du]
#          + 0.5 [dx;du]' [Qxx Qxu; Qxu' Quu] [dx;du]
float64 action_constant
float64[] action_state_gradient
float64[] action_control_gradient
float64[] action_state_hessian
float64[] action_state_control_hessian
float64[] action_control_hessian

# Contacts active in the source action model.
string[] active_contacts

# Numerical regularization reported by the upper solver. The matrices above
# are transported as supplied by the solver; consumers must not silently add
# this value as physical curvature.
float64 regularization
