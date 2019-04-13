function C = constraint_dt_dyn(revolute, simple, t, q)

r_len = length(revolute);
s_len = length(simple);


n_constr = 2 * r_len + s_len;

C = zeros(n_constr, 1);

c_idx = 2 * r_len + s_len;
