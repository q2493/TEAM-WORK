function C = constraint_dtt(revolute, simple, driving, t, q, qp)

r_len = length(revolute);
s_len = length(simple);
d_len = length(driving);

n_constr = 2 * r_len + s_len + d_len;

C = zeros(n_constr, 1);

c_idx = 0;
for r = revolute
    C(c_idx + (1:2)) = revolute_joint_dtt(r.i, r.j, r.s_i, r.s_j, q, qp);
    c_idx = c_idx + 2;
end

c_idx = 2 * r_len + s_len;
for d = driving
    c_idx = c_idx + 1;
    C(c_idx) = driving_joint_dtt(d.d_k_tt, t);
end