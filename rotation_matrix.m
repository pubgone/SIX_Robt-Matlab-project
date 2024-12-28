    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%2022.5.19%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%定义计算旋转矩阵的函数%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%liutangplease%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function r_matrix=rotation_matrix(axis,theta)
    axis = axis ./ norm(axis);  %将旋转轴变为单位向量
    a = cos(theta / 2);
    bcd =  -axis .* sin(theta / 2);
    b = bcd(1);
    c = bcd(2);
    d = bcd(3);
    aa = a^2;   bb = b^2;   cc = c^2;   dd = d^2;
    bc = b * c; ad = a * d; ac = a * c; ab = a*b; bd = b * d; cd = c * d;
    r_matrix = [(aa + bb - cc - dd)  (2 * (bc + ad))  (2 * (bd - ac)) ; (2 * (bc - ad))  (aa + cc - bb - dd)  (2 * (cd + ab)) ; (2 * (bd + ac))  (2 * (cd - ab))  (aa + dd - bb - cc)];
end