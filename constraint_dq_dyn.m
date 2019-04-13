function Cq = constraint_dq_dyn(revolute, simple, t, q)

r_len = length(revolute);
s_len = length(simple);

n_constr = 2 * r_len + s_len;

Cq = zeros(n_constr, length(q+1));

c_idx = 0;
for r = revolute
    Cq(c_idx + (1:2), [body_idx(r.i), body_idx(r.j)]) = ...
        revolute_joint_dq(r.i, r.j, r.s_i, r.s_j, q);
    c_idx = c_idx + 2;
end

for s = simple
    c_idx = c_idx + 1;
    Cq(c_idx, body_idx(s.i)) = simple_joint_dq(s.k);
end
