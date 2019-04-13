function g = revolute_joint_dtt(i, j, s_i, s_j, q, qp)

idx_i = body_idx(i);
r_i = q(idx_i(1:2));
phi_i = q(idx_i(3));
idx_j = body_idx(j);
r_j = q(idx_j(1:2));
phi_j = q(idx_j(3));

phi_p_i = qp(idx_i(3));
phi_p_j = qp(idx_j(3));

g = rot(phi_i) * s_i * phi_p_i^2 - rot(phi_j) * s_j * phi_p_j^2;