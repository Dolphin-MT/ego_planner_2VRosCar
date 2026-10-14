#include <iostream>
#include <traj_utils/polynomial_traj.h>

//多段 min-snap 轨迹生成
PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time)
{
  int seg_num = Time.size(); //待拟合的段数
  Eigen::MatrixXd poly_coeff(seg_num, 3 * 6); //每段轨迹的多项式系数矩阵，行数为段数，列数为每段轨迹的系数个数（3个轴方向，每个方向6个系数）六个系数分别对应五次多项式的六个系数
  Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);//系数向量 分别储存x y z三个方向的所有段的多项式系数

  int num_f, num_p; // number of fixed and free variables 固定约束 自由约束的个数
  int num_d;        // number of all segments' derivatives 端点导数约束的总维度

  //Factorial(x)=x! 阶乘
  const static auto Factorial = [](int x) {
    int fac = 1;
    for (int i = x; i > 0; i--)
      fac = fac * i;
    return fac;
  };

  /* ---------- end point derivative 构建端掉约束向量D 每个包含起点和终点的位置 速度 加速度---------- */
  Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
  Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    /* position to derivative k = 0~seg_num-1 */
    Dx(k * 6) = Pos(0, k);        // p_k(0) = 第k段起点x坐标
    Dx(k * 6 + 1) = Pos(0, k + 1);// p_k(T) = 第k段终点x位置
    Dy(k * 6) = Pos(1, k);
    Dy(k * 6 + 1) = Pos(1, k + 1);
    Dz(k * 6) = Pos(2, k);
    Dz(k * 6 + 1) = Pos(2, k + 1);

    //只有第一段和最后一段的边界约束 
    //中间段的速度和加速度不在此处直接指定，它们作为连续性约束通过选择矩阵C来处理
    if (k == 0) //第一段
    {
      Dx(k * 6 + 2) = start_vel(0);//p'(0)=起点速度 三个轴方向
      Dy(k * 6 + 2) = start_vel(1);
      Dz(k * 6 + 2) = start_vel(2);

      Dx(k * 6 + 4) = start_acc(0); //p''(0)=起点加速度
      Dy(k * 6 + 4) = start_acc(1);
      Dz(k * 6 + 4) = start_acc(2);
    }
    else if (k == seg_num - 1) //最后一段
    {
      Dx(k * 6 + 3) = end_vel(0);//p'(T)=终点速度
      Dy(k * 6 + 3) = end_vel(1);
      Dz(k * 6 + 3) = end_vel(2);

      Dx(k * 6 + 5) = end_acc(0);//p''(T)=终点加速度
      Dy(k * 6 + 5) = end_acc(1);
      Dz(k * 6 + 5) = end_acc(2);
    }
  }
  //最后Dx的状态，y z同理 Dx = [p₀(0), p₀(T₀), p'₀(0), p'₀(T₀), p''₀(0), p''₀(T₀)]T 边界约束
  /* ---------- Mapping Matrix A 构建映射矩阵 A 
  * A是块对角矩阵，每个6×6块Ab将多项式系数映射到端点导数
  * 核心关系 D=A*P P是所有段的多项式系数向量
  */
  Eigen::MatrixXd Ab;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    Ab = Eigen::MatrixXd::Zero(6, 6);
    for (int i = 0; i < 3; i++) // i=0,1,2 对应位置、速度、加速度
    {     //Factorial()，意味i的阶乘
      Ab(2 * i, i) = Factorial(i);
      for (int j = i; j < 6; j++)
        Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);//Time(k)就是第K段轨迹时长
    }
    A.block(k * 6, k * 6, 6, 6) = Ab; // 放到大矩阵的块对角线上
  }

  /* ---------- Produce Selection Matrix C' 构建选择矩阵C---------- */
  Eigen::MatrixXd Ct, C;
  //固定约束
  num_f = 2 * seg_num + 4; // 固定约束数 起点PVA(3) + 终点PVA(3) + 中间点位置(2(m-1)) =  3 + 3 + (seg_num - 1) * 2 = 2m + 4
  //自由
  num_p = 2 * seg_num - 2; // 自由变量数 中间点的速度(2(m-1)) + 中间点的加速度(0)  = (seg_num - 1) * 2 = 2m - 2
  num_d = 6 * seg_num;
  Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
  //起点约束 位置 速度 加速度 
  Ct(0, 0) = 1;
  Ct(2, 1) = 1;
  Ct(4, 2) = 1; // stack the start point
  //第二段起点位的一段的终点
  Ct(1, 3) = 1; //第1段终点位置
  Ct(3, 2 * seg_num + 4) = 1;
  Ct(5, 2 * seg_num + 5) = 1;
  //最后一段的约束
  Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
  Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
  Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
  Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

  //中间段的核心原理：位置约束是固定的，速度和加速度约束是自由的且前后段共享同一个自由变量
  for (int j = 2; j < seg_num; j++)//中间段
  {
    Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1; //起点位置
    Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1; //终点位置
    Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;//起点速度
    Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;//终点速度
    Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;//加速度
    Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
  }
//transpose()求转置
  C = Ct.transpose();

  Eigen::VectorXd Dx1 = C * Dx;
  Eigen::VectorXd Dy1 = C * Dy;
  Eigen::VectorXd Dz1 = C * Dz;

  /* ---------- minimum snap matrix 构建代价矩阵Q
    把"轨迹光滑程度"变成了可以用数字计算的"代价"。
  */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

  for (int k = 0; k < seg_num; k++)
  {
    //只有 i,j≥3  的位置有值，因为只有c3,c4,c5贡献 jerk。
    for (int i = 3; i < 6; i++)
    {
      for (int j = 3; j < 6; j++)
      {
        Q(k * 6 + i, k * 6 + j) =
            i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
      }
    }
  }

  /* ---------- R matrix 把"带约束的优化问题"变成"无约束的求导=0问题"，直接算出答案---------- */
  /* inverse() 矩阵求逆
  * segment(start,len) 对向量切片，提取从start开始len长度的子向量
  * block():
    R.block(0, 0, rows, cols) 表示从第 0 行第 0 列开始，取rows × cols 的子矩阵，而且是零拷贝视图。
  */
  Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct; //R = C*A(转置后求逆)*Q*A(逆)*C(转置)

  Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);
  //提取固定约束
  Dxf = Dx1.segment(0, 2 * seg_num + 4);
  Dyf = Dy1.segment(0, 2 * seg_num + 4);
  Dzf = Dz1.segment(0, 2 * seg_num + 4);
  /*将R矩阵分块为
  Rff Rfp
  Rpf Rpp*/
  Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
  Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
  Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
  Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

  Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
  Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
  Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
  Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

  /* ---------- close form solution 闭式求解自由变量 
  * 对代价函数求导并令∂Dp/∂J=0
  * Dp=−Rpp−1*RfpT*Df=−Rpp−1*Rpf*Df  (说明 
  * -1为求逆矩阵 T为求转置矩阵)
  */
  
  Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
  Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
  Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
  Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;
  //求解出的矩阵塞进D1
  Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
  Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
  Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;
  //还原多样式系数 P=A-1*CT*D1
  Px = (A.inverse() * Ct) * Dx1;
  Py = (A.inverse() * Ct) * Dy1;
  Pz = (A.inverse() * Ct) * Dz1;
  //整体系统并储存 将展平的系数向量转化为矩阵形式
  for (int i = 0; i < seg_num; i++)
  {
    poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
    poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
  }

  /* ---------- use polynomials 储存到polynomials---------- */
  PolynomialTraj poly_traj;
  //rows():取行数。用行数作为循环上界，保证每段多项式恰好被处理一次
  for (int i = 0; i < poly_coeff.rows(); ++i)
  {
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j)
    {// 提取每段系数（注意顺序是 [c_5, c_4, c_3, c_2, c_1, c_0]）
      cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
    }
    // 反转为 [c_0, c_1, ..., c_5]（PolynomialTraj 内部存储顺序）
    //reverse(x,y)STL算法 将向量元素顺序反转
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = Time(i);
    poly_traj.addSegment(cx, cy, cz, ts); //addSegment() 向poly_traj对象添加一段轨迹
  }

  return poly_traj;
}
/*功能：生成一条单段 5 次多项式轨迹，满足起点和终点的位置、速度、加速度共 6 个边界条件
  p(τ) = c₅τ⁵ + c₄τ⁴ + c₃τ³ + c₂τ² + c₁τ + c₀,  τ∈[0,t]
*/
PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                                    double t)
{
  //构建约束矩阵
  //C：6×6 约束矩阵  Crow：临时行向量1*6  Bx、By、Bz：x y z方向的 6×1 约束向量
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6), Crow(1, 6);
  Eigen::VectorXd Bx(6), By(6), Bz(6);
  //填充约束矩阵
  C(0, 5) = 1;//第0行：0 0 0 0 0 1
  C(1, 4) = 1;//第1行：0 0 0 0 1 0
  C(2, 3) = 2;//第2行：0 0 0 2 0 0
  Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;//0价导数
  C.row(3) = Crow;//C.row(3)即为C的第3行 t^5 t^4 t^3 t^2 t 1
  Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;//1价导数
  C.row(4) = Crow;//C.row(4)即为C的第4行 5t^4 4t^3 3t^2 2t 1 0
  Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;//2价导数
  C.row(5) = Crow;//C.row(5)即为C的第5行 20t^3 12t^2 6t 2 0 0
  //构建边界条件向量 三个轴方向的位置 速度 加速度分量
  Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
  By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
  Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);
  //求解线性方程组 C⋅cx=Bx 前三行是起点约束 后三行是终点约束，cx为多项式系数向量
  //QR 分解（ colPivHouseholderQr ）求解 三个轴方向的求解记为 Cofx Cofy Cofz 1*6矩阵
  Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
  Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);
  Eigen::VectorXd Cofz = C.colPivHouseholderQr().solve(Bz);

  vector<double> cx(6), cy(6), cz(6);
  for (int i = 0; i < 6; i++)
  {
    cx[i] = Cofx(i);
    cy[i] = Cofy(i);
    cz[i] = Cofz(i);
  }

  PolynomialTraj poly_traj;
  poly_traj.addSegment(cx, cy, cz, t);

  return poly_traj;
}
