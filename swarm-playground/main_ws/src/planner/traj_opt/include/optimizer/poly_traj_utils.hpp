#pragma once

#include "root_finder.hpp"

#include <iostream>
#include <cmath>
#include <vector>

#include <Eigen/Eigen>

namespace poly_traj
{

    // Polynomial order and trajectory dimension are fixed here
    typedef Eigen::Matrix<double, 3, 6> CoefficientMat;//定义类型别名 CoefficientMat。它是一个3x6的矩阵，表示三维五阶多项式的系数
    typedef Eigen::Matrix<double, 3, 5> VelCoefficientMat;//速度多项式系数矩阵，3 x 5。因为位置是五阶，求导后速度变成四阶，所以只有 5 个系数
    typedef Eigen::Matrix<double, 3, 4> AccCoefficientMat;//加速度多项式系数矩阵，3 x 4。速度再求导一次，加速度是三阶，所以有 4 个系数

    class Piece//定义类 Piece，表示“一段轨迹”
    {
    private:
        double duration;//这段轨迹持续的时间长度
        CoefficientMat coeffMat;//这一段轨迹的多项式系数矩阵

    public:
        Piece() = default;

        Piece(double dur, const CoefficientMat &cMat)
            : duration(dur), coeffMat(cMat) {}

        inline int getDim() const
        {
            return 3;//返回轨迹维度是 3，也就是三维轨迹
        }

        inline int getOrder() const
        {
            return 5;//返回多项式阶数是 5，也就是五阶轨迹
        }

        inline double getDuration() const
        {
            return duration;//返回这段轨迹的时长
        }

        inline const CoefficientMat &getCoeffMat() const
        {
            return coeffMat;//返回系数矩阵的常量引用
        }

        inline VelCoefficientMat getVelCoeffMat() const//计算并返回“速度多项式”的系数矩阵
        {
            VelCoefficientMat velCoeffMat;
            int n = 1;
            for (int i = 4; i >= 0; i--)
            {
                velCoeffMat.col(i) = n * coeffMat.col(i);//对位置多项式求导
                n++;
            }
            return velCoeffMat;
        }

        inline Eigen::Vector3d getPos(const double &t) const//给定时间 t，计算该时刻的三维位置
        {
            Eigen::Vector3d pos(0.0, 0.0, 0.0);//初始化位置为零向量
            double tn = 1.0;
            for (int i = 5; i >= 0; i--)
            {
                pos += tn * coeffMat.col(i);//把当前项加到位置上，相当于：先加 1 * 常数项，再加 t * 一次项，再加 t^2 * 二次项...最后加 t^5 * 五次项
                tn *= t;
            }
            return pos;
        }

        inline Eigen::Vector3d getVel(const double &t) const//给定时间 t，计算速度
        {
            Eigen::Vector3d vel(0.0, 0.0, 0.0);//初始化速度为零向量
            double tn = 1.0;
            int n = 1;
            for (int i = 4; i >= 0; i--)
            {
                vel += n * tn * coeffMat.col(i);//例如原位置：p(t) = c0 + c1 t + c2 t^2 + ...，则速度：v(t) = c1 + 2 c2 t + 3 c3 t^2 + ...
                tn *= t;
                n++;
            }
            return vel;
        }

        inline Eigen::Vector3d getAcc(const double &t) const//给定时间 t，计算加速度
        {
            Eigen::Vector3d acc(0.0, 0.0, 0.0);
            double tn = 1.0;
            int m = 1;
            int n = 2;
            for (int i = 3; i >= 0; i--)
            {
                acc += m * n * tn * coeffMat.col(i);//a(t) = 2 c2 + 6 c3 t + 12 c4 t^2 + 20 c5 t^3
                tn *= t;
                m++;
                n++;
            }
            return acc;
        }

        inline Eigen::Vector3d getJer(const double &t) const//给定时间 t，计算加加速度
        {
            Eigen::Vector3d jer(0.0, 0.0, 0.0);
            double tn = 1.0;
            int l = 1;
            int m = 2;
            int n = 3;
            for (int i = 2; i >= 0; i--)
            {
                jer += l * m * n * tn * coeffMat.col(i);//j(t) = 6 c3 + 24 c4 t + 60 c5 t^2
                tn *= t;
                l++;
                m++;
                n++;
            }
            return jer;
        }

        inline CoefficientMat normalizePosCoeffMat() const//把位置多项式从“真实时间版本”改成“归一化时间版本”
        {
            CoefficientMat nPosCoeffsMat;
            double t = 1.0;
            for (int i = 5; i >= 0; i--)//从 col(5) 到 col(0) 遍历位置多项式的 6 列系数，col(5) 是常数项，col(4) 是一次项...col(0) 是五次项
            {
                nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
                t *= duration;
            }
            return nPosCoeffsMat;
        }

        inline VelCoefficientMat normalizeVelCoeffMat() const//返回归一化后的速度系数矩阵
        {
            VelCoefficientMat nVelCoeffMat;
            int n = 1;
            double t = duration;
            for (int i = 4; i >= 0; i--)
            {
                nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
                t *= duration;
                n++;
            }
            return nVelCoeffMat;
        }

        inline AccCoefficientMat normalizeAccCoeffMat() const//返回归一化后的加速度系数矩阵
        {
            AccCoefficientMat nAccCoeffMat;
            int n = 2;
            int m = 1;
            double t = duration * duration;
            for (int i = 3; i >= 0; i--)
            {
                nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
                n++;
                m++;
                t *= duration;
            }
            return nAccCoeffMat;
        }

        inline double getMaxVelRate() const//定义函数，返回这段轨迹上的最大速度标量
        {
            Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();//先拿到归一化后的速度系数矩阵
            Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(2));//构造“速度平方”的多项式||v(s)||^2 = vx(s)^2 + vy(s)^2 + vz(s)^2
            int N = coeff.size();//记录这个多项式一共有多少个系数
            int n = N - 1;//多项式最高次幂
            for (int i = 0; i < N; i++)//对 ||v(s)||^2 求导
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)//导数几乎是零多项式
            {
                return 0.0;
            }
            else//导数不是零多项式
            {
                double l = -0.0625;
                double r = 1.0625;//设定求根区间，稍微比 [0,1] 大一点，这样做是为了数值稳定，避免根刚好卡在边界上
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;//如果左边界 l 恰好让导数值接近 0，就继续往外挪一点，避免求根器在边界处不稳定
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                          FLT_EPSILON / duration);//对导数多项式求根，得到所有候选极值点
                candidates.insert(0.0);
                candidates.insert(1.0);//把区间两端也加进去。因为最大值不一定在内部极值点，也可能出现在端点
                double maxVelRateSqr = -INFINITY;//初始化“最大速度平方”为负无穷
                double tempNormSqr;//临时变量，用来存某个候选点上的速度平方
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)//遍历所有候选点
                {
                    if (0.0 <= *it && 1.0 >= *it)//只保留归一化时间范围 [0,1] 内的点
                    {
                        tempNormSqr = getVel((*it) * duration).squaredNorm();//把归一化时间 s 还原成真实时间 t = s * duration，然后调用 getVel(t) 计算真实速度，再取平方范数
                        maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;//如果当前点速度更大，就更新最大值
                    }
                }
                return sqrt(maxVelRateSqr);//开根号，返回真实最大速度
            }
        }

        inline double getMaxAccRate() const//定义函数，返回这段轨迹上的最大加速度标量
        {
            Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(2));
            int N = coeff.size();
            int n = N - 1;
            for (int i = 0; i < N; i++)
            {
                coeff(i) *= n;
                n--;
            }
            if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
            {
                return 0.0;
            }
            else
            {
                double l = -0.0625;
                double r = 1.0625;
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
                {
                    l = 0.5 * l;
                }
                while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
                {
                    r = 0.5 * (r + 1.0);
                }
                std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                          FLT_EPSILON / duration);
                candidates.insert(0.0);
                candidates.insert(1.0);
                double maxAccRateSqr = -INFINITY;
                double tempNormSqr;
                for (std::set<double>::const_iterator it = candidates.begin();
                     it != candidates.end();
                     it++)
                {
                    if (0.0 <= *it && 1.0 >= *it)
                    {
                        tempNormSqr = getAcc((*it) * duration).squaredNorm();
                        maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                    }
                }
                return sqrt(maxAccRateSqr);
            }
        }
        //检查这段轨迹的速度是否始终小于给定上限
        inline bool checkMaxVelRate(const double &maxVelRate) const//输入允许的最大速度 maxVelRate，返回布尔值
        {
            double sqrMaxVelRate = maxVelRate * maxVelRate;//先把最大速度平方一下
            if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
                getVel(duration).squaredNorm() >= sqrMaxVelRate)
            {
                return false;//如果起点或终点超限，直接返回 false
            }
            else
            {
                Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
                Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                        RootFinder::polySqr(nVelCoeffMat.row(2));
                double t2 = duration * duration;
                coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;//统计这个方程在 [0,1] 内有没有根，没有根：说明全程都严格低于限制，返回 true，有根：说明中途碰到阈值，甚至可能超限，返回 false
            }
        }
        //检查这段轨迹的加速度是否始终小于给定上限
        inline bool checkMaxAccRate(const double &maxAccRate) const
        {
            double sqrMaxAccRate = maxAccRate * maxAccRate;
            if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
                getAcc(duration).squaredNorm() >= sqrMaxAccRate)
            {
                return false;
            }
            else
            {
                Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
                Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                        RootFinder::polySqr(nAccCoeffMat.row(2));
                double t2 = duration * duration;
                double t4 = t2 * t2;
                coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
                return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
            }
        }

        // GaaiLam，给定一个空间点 pt，把它“投影”到这段多项式轨迹上，找到轨迹上距离它最近的点 pro_pt，以及该点对应的时间 tt
        inline bool project_pt(const Eigen::Vector3d &pt,
                               double &tt, Eigen::Vector3d &pro_pt)
        {
            // 2*(p-p0)^T * \dot{p} = 0，f(t) = ||p(t) - pt||^2对时间求导f'(t) = 2 (p(t)-pt)^T p'(t)= 0
            auto l_coeff = getCoeffMat();//取出当前位置多项式的系数矩阵
            l_coeff.col(5) = l_coeff.col(5) - pt;
            auto r_coeff = getVelCoeffMat();//取出速度多项式 p'(t) 的系数矩阵
            Eigen::VectorXd eq = Eigen::VectorXd::Zero(2 * 5);
            for (int j = 0; j < l_coeff.rows(); ++j)
            {
                eq = eq + RootFinder::polyConv(l_coeff.row(j), r_coeff.row(j));//最后 eq 就是(p(t)-pt)^T p'(t)对应的系数表达
            }
            double l = -0.0625;
            double r = duration + 0.0625;
            while (fabs(RootFinder::polyVal(eq, l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(eq, r)) < DBL_EPSILON)
            {
                r = 0.5 * (duration + 1.0);
            }
            std::set<double> roots =
                RootFinder::solvePolynomial(eq, l, r, 1e-6);//对方程(p(t)-pt)^T p'(t) = 0进行求根，得到的 roots 是所有候选时间点，也就是“距离平方函数可能取得极值”的时刻
            // std::cout << "# roots: " << roots.size() << std::endl;
            double min_dist = -1;
            for (const auto &root : roots)//遍历所有候选根，也就是所有可能的投影时刻
            {
                // std::cout << "root: " << root << std::endl;
                if (root < 0 || root > duration)
                {
                    continue;
                }
                if (getVel(root).norm() < 1e-6)
                { // velocity == 0, ignore it
                    continue;
                }
                // std::cout << "find min!" << std::endl;
                Eigen::Vector3d p = getPos(root);
                // std::cout << "p: " << p.transpose() << std::endl;
                double distance = (p - pt).norm();//计算当前候选轨迹点到目标点 pt 的欧氏距离
                if (distance < min_dist || min_dist < 0)//当前这个候选点，比之前记录的最近点还要近或者目前还没有找到任何合法候选点，就更新“当前最优解”
                {
                    min_dist = distance;
                    tt = root;
                    pro_pt = p;
                }
            }
            return min_dist > 0;
        }
        //给定一个平面，求这段轨迹和这个平面的交点，p：平面上的一点，v：平面法向量，tt：输出，交点对应的时间，pt：输出，交点位置
        inline bool intersection_plane(const Eigen::Vector3d p,
                                       const Eigen::Vector3d v,
                                       double &tt, Eigen::Vector3d &pt) const
        {
            // (pt - p)^T * v = 0，轨迹点减去平面上一点，再和法向量点乘，等于 0，就说明这个点在平面上
            auto coeff = getCoeffMat();
            coeff.col(5) = coeff.col(5) - p;
            Eigen::VectorXd eq = coeff.transpose() * v;
            double l = -0.0625;
            double r = duration + 0.0625;
            while (fabs(RootFinder::polyVal(eq, l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(eq, r)) < DBL_EPSILON)
            {
                r = 0.5 * (duration + 1.0);
            }
            std::set<double> roots =
                RootFinder::solvePolynomial(eq, l, r, 1e-6);
            for (const auto &root : roots)
            {
                tt = root;
                pt = getPos(root);
                return true;//一旦找到第一个根，立刻返回 true
            }
            return false;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    class Trajectory//由多段 Piece 拼起来的一整条轨迹
    {
    private:
        typedef std::vector<Piece> Pieces;
        Pieces pieces;

    public:
        Trajectory() = default;
        //构造出一个新的轨迹段，并放进容器里
        Trajectory(const std::vector<double> &durs,//durs：每一段轨迹的持续时间
                   const std::vector<CoefficientMat> &cMats)//cMats：每一段轨迹对应的系数矩阵
        {
            int N = std::min(durs.size(), cMats.size());//取两个输入数组长度的较小值，记为 N，避免越界访问
            pieces.reserve(N);
            for (int i = 0; i < N; i++)
            {
                pieces.emplace_back(durs[i], cMats[i]);
            }
        }

        inline int getPieceNum() const//返回这条轨迹一共有多少段 Piece
        {
            return pieces.size();
        }

        inline Eigen::VectorXd getDurations() const//返回所有轨迹段时长组成的向量
        {
            int N = getPieceNum();
            Eigen::VectorXd durations(N);
            for (int i = 0; i < N; i++)
            {
                durations(i) = pieces[i].getDuration();
            }
            return durations;
        }

        inline double getTotalDuration() const//返回整条轨迹的总时长
        {
            int N = getPieceNum();
            double totalDuration = 0.0;
            for (int i = 0; i < N; i++)
            {
                totalDuration += pieces[i].getDuration();
            }
            return totalDuration;
        }

        inline Eigen::MatrixXd getPositions() const//返回一个矩阵，里面存的是整条轨迹的“关键位置点”
        {
            int N = getPieceNum();
            Eigen::MatrixXd positions(3, N + 1);//创建一个 3 x (N+1) 的矩阵 positions
            for (int i = 0; i < N; i++)
            {
                positions.col(i) = pieces[i].getCoeffMat().col(5);//放入了每一段的起点
            }
            positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());//把最后一列设为“最后一段的终点位置“
            return positions;
        }

        inline const Piece &operator[](int i) const//只读版本
        {
            return pieces[i];
        }

        inline Piece &operator[](int i)//直接拿到第 i 段轨迹，并允许修改它
        {
            return pieces[i];
        }

        inline void clear(void)
        {
            pieces.clear();
            return;
        }

        inline Pieces::const_iterator begin() const//返回一个只读迭代器，指向 pieces 的开头
        {
            return pieces.begin();
        }

        inline Pieces::const_iterator end() const//返回一个只读迭代器，指向 pieces 末尾之后的位置
        {
            return pieces.end();
        }

        inline Pieces::iterator begin()
        {
            return pieces.begin();
        }

        inline Pieces::iterator end()
        {
            return pieces.end();
        }

        inline void reserve(const int &n)//输入 n，表示想提前预留多少段轨迹的空间
        {
            pieces.reserve(n);
            return;
        }

        inline void emplace_back(const Piece &piece)
        {
            pieces.emplace_back(piece);//把这个 Piece 添加到 pieces 容器末尾，相当于“往这条 Trajectory 后面再接一段轨迹”
            return;
        }

        inline void emplace_back(const double &dur,
                                 const CoefficientMat &cMat)
        {
            pieces.emplace_back(dur, cMat);//直接在 pieces 尾部原地构造一个 Piece(dur, cMat)
            return;
        }

        inline void append(const Trajectory &traj)//把另一条traj追加到当前轨迹后面。
        {
            pieces.insert(pieces.end(), traj.begin(), traj.end());
            return;
        }

        inline int locatePieceIdx(double &t) const//给一个整条轨迹上的时间t，判断这个时间落在哪一段Piece里，同时把t原地改成“这一段内部的局部时间
        {
            int N = getPieceNum();
            int idx;
            double dur;
            for (idx = 0;
                 idx < N &&
                 t > (dur = pieces[idx].getDuration());
                 idx++)
            {
                t -= dur;
            }
            if (idx == N)
            {
                idx--;
                t += pieces[idx].getDuration();
            }
            return idx;//返回值 idx：是哪一段
        }

        inline Eigen::Vector3d getPos(double t) const//把全局时间映射到某一段，再求这一段上的位置
        {
            int pieceIdx = locatePieceIdx(t);//locatePieceIdx 负责“整条轨迹时间 -> 某一段 + 段内时间
            return pieces[pieceIdx].getPos(t);//Piece::getPos 负责“在这一段上根据局部时间算位置
        }

        inline Eigen::Vector3d getVel(double t) const//输入整条轨迹上的时间 t，返回该时刻速度
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getVel(t);
        }

        inline Eigen::Vector3d getAcc(double t) const//输入全局时间 t，返回该时刻加速度
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getAcc(t);
        }

        inline Eigen::Vector3d getJer(double t) const//输入全局时间 t，返回该时刻 jerk
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getJer(t);
        }

        inline Eigen::Vector3d getJuncPos(int juncIdx) const//输入连接点编号 juncIdx，返回这个连接点的三维位置
        {
            if (juncIdx != getPieceNum())//判断当前取的连接点是不是“最后一个连接点
            {
                return pieces[juncIdx].getCoeffMat().col(5);
            }
            else
            {
                return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());//整条轨迹的终点位置
            }
        }

        inline Eigen::Vector3d getJuncVel(int juncIdx) const//输入连接点编号，返回该连接点的速度
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(4);
            }
            else
            {
                return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::Vector3d getJuncAcc(int juncIdx) const//输入连接点编号，返回该点加速度
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(3) * 2.0;
            }
            else
            {
                return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
            }
        }

        inline double getMaxVelRate() const//返回整条 Trajectory 的最大速度大小
        {
            int N = getPieceNum();
            double maxVelRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxVelRate();
                maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
            }
            return maxVelRate;
        }

        inline double getMaxAccRate() const//返回整条轨迹的最大加速度
        {
            int N = getPieceNum();
            double maxAccRate = -INFINITY;
            double tempNorm;
            for (int i = 0; i < N; i++)
            {
                tempNorm = pieces[i].getMaxAccRate();
                maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
            }
            return maxAccRate;
        }

        inline bool checkMaxVelRate(const double &maxVelRate) const//检查整条轨迹是否满足最大速度限制 maxVelRate
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);//只有当：前面所有段都满足，当前这一段也满足，最终 feasible 才会继续保持 true
            }
            return feasible;
        }

        inline bool checkMaxAccRate(const double &maxAccRate) const//检查整条轨迹是否满足加速度上限
        {
            int N = getPieceNum();
            bool feasible = true;
            for (int i = 0; i < N && feasible; i++)
            {
                feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
            }
            return feasible;
        }

        // GaaiLam
        inline Piece getPiece(int i) const//输入第 i 段的编号，返回这一段 Piece
        {
            return pieces[i];
        }
        //在整条分段轨迹里，找点 pt 在哪一段上可以投影成功，并返回：ii：投影落在哪一段，tt：该段内对应的时间，pro_pt：投影点坐标
        inline bool project_pt(const Eigen::Vector3d &pt,
                               int &ii, double &tt, Eigen::Vector3d &pro_pt)
        {
            bool find_project_pt = false;
            for (int i = 0; i < getPieceNum(); ++i)
            {
                auto piece = pieces[i];
                if (piece.project_pt(pt, tt, pro_pt))
                {
                    ii = i;
                    find_project_pt = true;
                    break;
                }
            }
            if (!find_project_pt)
            {
                // std::cout << "\033[32m" << "cannot project pt to traj" << "\033[0m" << std::endl;
                // std::cout << "pt: " << pt.transpose() << std::endl;
                // assert(false);
            }
            return find_project_pt;
        }
        //在整条分段轨迹中，寻找与某个平面的交点，输入p：平面上一点，v：平面法向量
        inline bool intersection_plane(const Eigen::Vector3d p,
                                       const Eigen::Vector3d v,
                                       int &ii, double &tt, Eigen::Vector3d &pt)
        {
            for (int i = 0; i < getPieceNum(); ++i)
            {
                const auto &piece = pieces[i];
                if (piece.intersection_plane(p, v, tt, pt))
                {
                    ii = i;
                    return true;//找到第一个交点就结束
                }
            }
            return false;
        }

        inline std::vector<Eigen::Vector3d> way_points()//取每一段轨迹的起点（不包含最终终点）
        {
            std::vector<Eigen::Vector3d> pts;
            for (int i = 0; i < getPieceNum(); ++i)
            {
                pts.push_back(pieces[i].getPos(0));//把第 i 段轨迹在局部时间 0 的位置加入 pts
            }
            return pts;
        }

        // zxzx
        inline std::pair<int, double> locatePieceIdxWithRatio(double &t) const//根据全局时间找到所在段，并算出段内比例
        {
            int N = getPieceNum();
            int idx;
            double dur;
            for (idx = 0;
                 idx < N &&
                 t > (dur = pieces[idx].getDuration());
                 idx++)
            {
                t -= dur;
            }
            if (idx == N)
            {
                idx--;
                t += pieces[idx].getDuration();
            }
            std::pair<int, double> idx_ratio;
            idx_ratio.first = idx;//保存轨迹段编号
            idx_ratio.second = t / dur;//保存段内比例
            return idx_ratio;
        }
//根据全局时间取位置，同时把“段号 + 段内比例”返回出去
        inline Eigen::Vector3d getPoswithIdxRatio(double t, std::pair<int, double> &idx_ratio) const
        {
            idx_ratio = locatePieceIdxWithRatio(t);
            return pieces[idx_ratio.first].getPos(t);
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    // The banded system class is used for solving
    // banded linear system Ax=b efficiently.这个类用于高效求解带状线性系统：A x = b
    // A is an N*N band matrix with lower band width lowerBw，矩阵 A 是一个 N x N 的带状矩阵，lowerBw：下带宽，表示主对角线下面最多有多少条非零对角线
    // and upper band width upperBw.upperBw：上带宽，表示主对角线上面最多有多少条非零对角线
    // Banded LU factorization has O(N) time complexity.带状矩阵做 LU 分解的复杂度可以接近 O(N)
    class BandedSystem
    {
    public:
        // The size of A, as well as the lower/upper创建这个系统时，需要知道矩阵大小 n，以及下带宽 p、上带宽 q
        // banded width p/q are needed
        inline void create(const int &n, const int &p, const int &q)//初始化带状矩阵存储空间
        {
            // In case of re-creating before destroying
            destroy();//如果这个对象之前已经分配过内存，先释放掉，避免内存泄漏
            N = n;
            lowerBw = p;
            upperBw = q;
            int actualSize = N * (lowerBw + upperBw + 1);//计算实际需要存储多少个 double
            ptrData = new double[actualSize];
            std::fill_n(ptrData, actualSize, 0.0);//矩阵初始全 0
            return;
        }

        inline void destroy()//释放 ptrData 指向的内存
        {
            if (ptrData != nullptr)
            {
                delete[] ptrData;
                ptrData = nullptr;
            }
            return;
        }

        inline void operator=(const BandedSystem &bs)//重载赋值"="运算符,从另一个 BandedSystem 拷贝矩阵数据
        {
            ptrData = nullptr;
            create(bs.N, bs.lowerBw, bs.upperBw);
            memcpy(ptrData, bs.ptrData, N * (lowerBw + upperBw + 1) * sizeof(double));//把源对象 bs.ptrData 里的数据拷贝到当前对象 ptrData
        }

    private:
        int N;
        int lowerBw;
        int upperBw;
        double *ptrData = nullptr;//裸指针，指向真正存储带状矩阵数据的一维数组,初始为 nullptr，表示还没有分配内存

    public:
        // Reset the matrix to zero把矩阵重置为全 0
        inline void reset(void)
        {
            std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
            return;
        }

        // The band matrix is stored as suggested in "Matrix Computation"
        inline const double &operator()(const int &i, const int &j) const//重载 () 运算符的只读版本
        {
            return ptrData[(i - j + upperBw) * N + j];//把矩阵下标 (i, j) 转换成一维数组下标
        }

        inline double &operator()(const int &i, const int &j)//() 运算符的可修改版本
        {
            return ptrData[(i - j + upperBw) * N + j];
        }

        // This function conducts banded LU factorization in place这个函数对带状矩阵做 LU 分解，A = L U,而且是在原矩阵上直接改
        // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!为了效率，没有做 pivot，也就是没有选主元、没有换行
        inline void factorizeLU()
        {
            int iM, jM;
            double cVl;
            for (int k = 0; k <= N - 2; k++)
            {
                iM = std::min(k + lowerBw, N - 1);
                cVl = operator()(k, k);
                for (int i = k + 1; i <= iM; i++)
                {
                    if (operator()(i, k) != 0.0)
                    {
                        operator()(i, k) /= cVl;
                    }
                }
                jM = std::min(k + upperBw, N - 1);
                for (int j = k + 1; j <= jM; j++)
                {
                    cVl = operator()(k, j);
                    if (cVl != 0.0)
                    {
                        for (int i = k + 1; i <= iM; i++)
                        {
                            if (operator()(i, k) != 0.0)
                            {
                                operator()(i, j) -= operator()(i, k) * cVl;
                            }
                        }
                    }
                }
            }
            return;
        }

        // This function solves Ax=b, then stores x in b求解 Ax=b，并把解 x 存回 b
        // The input b is required to be N*m, i.e.,b 必须是 N x m 的矩阵
        // m vectors to be solved.
        inline void solve(Eigen::MatrixXd &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)//前代，解 L y = b
            {
                iM = std::min(j + lowerBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)//回代，解 U x = y
            {
                b.row(j) /= operator()(j, j);
                iM = std::max(0, j - upperBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(i, j) != 0.0)
                    {
                        b.row(i) -= operator()(i, j) * b.row(j);
                    }
                }
            }
            return;
        }

        // This function solves ATx=b, then stores x in b,AT 表示A 的转置
        // The input b is required to be N*m, i.e.,
        // m vectors to be solved.
        inline void solveAdj(Eigen::MatrixXd &b) const
        {
            int iM;
            for (int j = 0; j <= N - 1; j++)//解 U^T y = b
            {
                b.row(j) /= operator()(j, j);
                iM = std::min(j + upperBw, N - 1);
                for (int i = j + 1; i <= iM; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            for (int j = N - 1; j >= 0; j--)//解 L^T x = y
            {
                iM = std::max(0, j - lowerBw);
                for (int i = iM; i <= j - 1; i++)
                {
                    if (operator()(j, i) != 0.0)
                    {
                        b.row(i) -= operator()(j, i) * b.row(j);
                    }
                }
            }
            return;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    class MinJerkOpt
    {
    public:
        inline void operator=(const MinJerkOpt &mjo)//重载赋值运算符
        {
            N = mjo.N;//拷贝轨迹段数
            headPVA = mjo.headPVA;//拷贝起点状态,包含pose,v,a
            tailPVA = mjo.tailPVA;//拷贝终点状态
            T1 = mjo.T1;//拷贝每一段轨迹的时间
            A = mjo.A;
            b = mjo.b;
            T2 = mjo.T2;
            T3 = mjo.T3;
            T4 = mjo.T4;
            T5 = mjo.T5;//拷贝时间的高次幂
            gdC = mjo.gdC;//拷贝关于多项式系数的梯度
        }
        ~MinJerkOpt() { A.destroy(); }

    private:
        int N;//轨迹段数
        Eigen::Matrix3d headPVA;
        Eigen::Matrix3d tailPVA;
        Eigen::VectorXd T1;
        BandedSystem A;
        Eigen::MatrixXd b;

        // Temp variables
        Eigen::VectorXd T2;//T2 = T1^2
        Eigen::VectorXd T3;//T3 = T1^3
        Eigen::VectorXd T4;
        Eigen::VectorXd T5;
        Eigen::MatrixXd gdC;

    private:
        template <typename EIGENVEC>//如果某一段时间 T 变长一点，jerk 代价 J 会怎么变，可以把 gdT(i) 理解成第 i 段时间变长一点，cost 会增加还是减少，增加多少
        inline void addGradJbyT(EIGENVEC &gdT) const//把 jerk 代价对时间的直接梯度加到 gdT，我把这一段时间拉长，积分区间变了，cost 怎么变
        {
            for (int i = 0; i < N; i++)
            {
                gdT(i) += 36.0 * b.row(6 * i + 3).squaredNorm() +
                          288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                          576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                          720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                          2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                          3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
            }
            return;
        }

        template <typename EIGENMAT>//如果某一段多项式系数 c3/c4/c5 变一点，jerk 代价 J 会怎么变，gdC = J 对多项式系数 C 的梯度
        inline void addGradJbyC(EIGENMAT &gdC) const//把 jerk 代价对多项式系数的梯度加到 gdC，我改多项式形状，cost 怎么变
        {
            for (int i = 0; i < N; i++)
            {
                gdC.row(6 * i + 5) += 240.0 * b.row(6 * i + 3) * T3(i) +
                                      720.0 * b.row(6 * i + 4) * T4(i) +
                                      1440.0 * b.row(6 * i + 5) * T5(i);//计算并累加对 c5 的梯度
                gdC.row(6 * i + 4) += 144.0 * b.row(6 * i + 3) * T2(i) +
                                      384.0 * b.row(6 * i + 4) * T3(i) +
                                      720.0 * b.row(6 * i + 5) * T4(i);
                gdC.row(6 * i + 3) += 72.0 * b.row(6 * i + 3) * T1(i) +
                                      144.0 * b.row(6 * i + 4) * T2(i) +
                                      240.0 * b.row(6 * i + 5) * T3(i);
            }
            return;
        }

        inline void solveAdjGradC(Eigen::MatrixXd &gdC) const//解伴随方程，传播系数梯度
        {
            A.solveAdj(gdC);//解的是A^T x = gdC
            return;
        }

        template <typename EIGENVEC>
        inline void addPropCtoT(const Eigen::MatrixXd &adjGdC, EIGENVEC &gdT) const//把“系数由时间变化间接引起的影响”继续加回时间梯度，最终得到的 gdT 才是完整的时间变化对轨迹代价的影响
        {
            Eigen::MatrixXd B1(6, 3), B2(3, 3);

            Eigen::RowVector3d negVel, negAcc, negJer, negSnp, negCrk;

            for (int i = 0; i < N - 1; i++)//中间连接约束只存在于相邻段之间，最后一段的终点约束后面单独处理。
            {
                negVel = -(b.row(i * 6 + 1) +
                           2.0 * T1(i) * b.row(i * 6 + 2) +
                           3.0 * T2(i) * b.row(i * 6 + 3) +
                           4.0 * T3(i) * b.row(i * 6 + 4) +
                           5.0 * T4(i) * b.row(i * 6 + 5));//算第 i 段终点速度的负值
                negAcc = -(2.0 * b.row(i * 6 + 2) +
                           6.0 * T1(i) * b.row(i * 6 + 3) +
                           12.0 * T2(i) * b.row(i * 6 + 4) +
                           20.0 * T3(i) * b.row(i * 6 + 5));
                negJer = -(6.0 * b.row(i * 6 + 3) +
                           24.0 * T1(i) * b.row(i * 6 + 4) +
                           60.0 * T2(i) * b.row(i * 6 + 5));
                negSnp = -(24.0 * b.row(i * 6 + 4) +
                           120.0 * T1(i) * b.row(i * 6 + 5));
                negCrk = -120.0 * b.row(i * 6 + 5);

                B1 << negSnp, negCrk, negVel, negVel, negAcc, negJer;//把这些向量填进 B1

                gdT(i) += B1.cwiseProduct(adjGdC.block<6, 3>(6 * i + 3, 0)).sum();
            }

            negVel = -(b.row(6 * N - 5) +
                       2.0 * T1(N - 1) * b.row(6 * N - 4) +
                       3.0 * T2(N - 1) * b.row(6 * N - 3) +
                       4.0 * T3(N - 1) * b.row(6 * N - 2) +
                       5.0 * T4(N - 1) * b.row(6 * N - 1));//对应最后一段的终点速度负值
            negAcc = -(2.0 * b.row(6 * N - 4) +
                       6.0 * T1(N - 1) * b.row(6 * N - 3) +
                       12.0 * T2(N - 1) * b.row(6 * N - 2) +
                       20.0 * T3(N - 1) * b.row(6 * N - 1));
            negJer = -(6.0 * b.row(6 * N - 3) +
                       24.0 * T1(N - 1) * b.row(6 * N - 2) +
                       60.0 * T2(N - 1) * b.row(6 * N - 1));

            B2 << negVel, negAcc, negJer;

            gdT(N - 1) += B2.cwiseProduct(adjGdC.block<3, 3>(6 * N - 3, 0)).sum();

            return;
        }

        template <typename EIGENMAT>
        inline void addPropCtoP(const Eigen::MatrixXd &adjGdC, EIGENMAT &gdInP) const//把梯度传回“中间路径点”
        {
            for (int i = 0; i < N - 1; i++)
            {
                // gdInP.col(i) += adjGdC.row(6 * i + 5).transpose();
                gdInP.col(i) = adjGdC.row(6 * i + 5).transpose(); // zxzx
            }
            return;
        }

        template <typename EIGENVEC>//给轨迹加“时间积分惩罚”，所谓时间积分惩罚，就是沿轨迹采样，把每个采样点的违规程度累计成代价
        inline void addTimeIntPenalty(const Eigen::VectorXi cons,//每段轨迹采样多少份
                                      const Eigen::VectorXi &idxHs,//每段轨迹对应哪个安全走廊/约束区域
                                      const std::vector<Eigen::MatrixXd> &cfgHs,//安全走廊半空间约束集合
                                      const double vmax,//最大速度
                                      const double amax,//最大加速度
                                      const Eigen::Vector3d ci,//三个惩罚权重，通常对应位置/速度/加速度
                                      double &cost,//总代价
                                      EIGENVEC &gdT,//时间梯度
                                      Eigen::MatrixXd &gdC) const//系数梯度
        {
            double pena = 0.0;
            const double vmaxSqr = vmax * vmax;
            const double amaxSqr = amax * amax;

            Eigen::Vector3d pos, vel, acc, jer;
            double step, alpha;
            double s1, s2, s3, s4, s5;
            Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3;
            Eigen::Vector3d outerNormal;//安全走廊某个半空间的外法向量
            int K;//当前安全走廊包含多少个半空间约束
            double violaPos, violaVel, violaAcc;//位置越界程度，速度超限程度，加速度超限程度
            double violaPosPenaD, violaVelPenaD, violaAccPenaD;
            double violaPosPena, violaVelPena, violaAccPena;
            Eigen::Matrix<double, 6, 3> gradViolaVc, gradViolaAc;//速度违规、加速度违规对多项式系数的梯度
            double gradViolaVt, gradViolaAt;//速度违规、加速度违规对时间的梯度
            double omg;//积分权重

            int innerLoop, idx;
            for (int i = 0; i < N; i++)
            {
                const auto &c = b.block<6, 3>(i * 6, 0);//取出第 i 段轨迹的 6 个三维系数
                step = T1(i) / cons(i);//计算采样步长
                s1 = 0.0;//局部时间从 0 开始
                innerLoop = cons(i) + 1;//如果分成 cons(i) 份，就有 cons(i)+1 个采样点，包括起点和终点
                for (int j = 0; j < innerLoop; j++)
                {
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    beta0 << 1.0, s1, s2, s3, s4, s5;
                    beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                    beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                    beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                    alpha = 1.0 / cons(i) * j;//alpha = j / cons(i)，当前采样点在本段里的归一化比例
                    pos = c.transpose() * beta0;
                    vel = c.transpose() * beta1;
                    acc = c.transpose() * beta2;
                    jer = c.transpose() * beta3;//计算采样点处的位置，速度，加速度，jerk
                    violaVel = vel.squaredNorm() - vmaxSqr;//计算速度和加速度是否超限
                    violaAcc = acc.squaredNorm() - amaxSqr;

                    omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;//第一个点和最后一个点权重 0.5，中间点权重 1.0

                    idx = idxHs(i);
                    K = cfgHs[idx].cols();//找到第 i 段对应的安全区域，并取出这个区域有多少个半空间约束
                    for (int k = 0; k < K; k++)
                    {
                        outerNormal = cfgHs[idx].col(k).head<3>();
                        violaPos = outerNormal.dot(pos - cfgHs[idx].col(k).tail<3>());
                        if (violaPos > 0.0)//只有违规才加惩罚
                        {
                            violaPosPenaD = violaPos * violaPos;
                            violaPosPena = violaPosPenaD * violaPos;
                            violaPosPenaD *= 3.0;//计算三次惩罚和导数
                            gdC.block<6, 3>(i * 6, 0) += omg * step * ci(0) * violaPosPenaD * beta0 * outerNormal.transpose();//把位置违规对系数 C 的梯度加到 gdC
                            gdT(i) += omg * (ci(0) * violaPosPenaD * alpha * outerNormal.dot(vel) * step +
                                             ci(0) * violaPosPena / cons(i));
                            pena += omg * step * ci(0) * violaPosPena;//把位置惩罚加到总惩罚项里
                        }
                    }
                    //速度约束惩罚
                    if (violaVel > 0.0)
                    {
                        violaVelPenaD = violaVel * violaVel;
                        violaVelPena = violaVelPenaD * violaVel;
                        violaVelPenaD *= 3.0;
                        gradViolaVc = 2.0 * beta1 * vel.transpose();
                        gradViolaVt = 2.0 * alpha * vel.transpose() * acc;
                        gdC.block<6, 3>(i * 6, 0) += omg * step * ci(1) * violaVelPenaD * gradViolaVc;
                        gdT(i) += omg * (ci(1) * violaVelPenaD * gradViolaVt * step +
                                         ci(1) * violaVelPena / cons(i));
                        pena += omg * step * ci(1) * violaVelPena;
                    }
                    //加速度约束惩罚
                    if (violaAcc > 0.0)
                    {
                        violaAccPenaD = violaAcc * violaAcc;
                        violaAccPena = violaAccPenaD * violaAcc;
                        violaAccPenaD *= 3.0;
                        gradViolaAc = 2.0 * beta2 * acc.transpose();
                        gradViolaAt = 2.0 * alpha * acc.transpose() * jer;
                        gdC.block<6, 3>(i * 6, 0) += omg * step * ci(2) * violaAccPenaD * gradViolaAc;
                        gdT(i) += omg * (ci(2) * violaAccPenaD * gradViolaAt * step +
                                         ci(2) * violaAccPena / cons(i));
                        pena += omg * step * ci(2) * violaAccPena;
                    }

                    s1 += step;
                }
            }

            cost += pena;//把所有采样点累计出来的惩罚加到总代价 cost 上
            return;
        }

    public:
        inline void reset(const Eigen::Matrix3d &headState,
                          const Eigen::Matrix3d &tailState,
                          const int &pieceNum)//准备内存和边界条件
        {
            N = pieceNum;
            headPVA = headState;
            tailPVA = tailState;
            T1.resize(N);
            A.create(6 * N, 6, 6);
            b.resize(6 * N, 3);
            gdC.resize(6 * N, 3);
            // gdT.resize(6 * N);
            return;
        }
        //根据起点、终点、中间点、时间，构造一个线性方程组
        //这个方程组表达这些约束：起点 PVA 固定,终点 PVA 固定,每段经过指定中间点,相邻段之间位置、速度、加速度、jerk、snap 连续
        //解这个线性方程组，得到每段五阶多项式的系数
        inline void generate(const Eigen::MatrixXd &inPs,//inPs：中间点，大小通常是 3 x (N-1)
                             const Eigen::VectorXd &ts)//ts：每段轨迹时间，长度是 N
        {
            if (inPs.cols() == 0)//没有中间点,说明只有一段轨迹：从起点直接到终点
            {

                T1(0) = ts(0);
                double t1_inv = 1.0 / T1(0);
                double t2_inv = t1_inv * t1_inv;
                double t3_inv = t2_inv * t1_inv;
                double t4_inv = t2_inv * t2_inv;
                double t5_inv = t4_inv * t1_inv;//计算：1/T, 1/T^2, ..., 1/T^5
                CoefficientMat coeffMatReversed;
                coeffMatReversed.col(5) = 0.5 * (tailPVA.col(2) - headPVA.col(2)) * t3_inv -
                                          3.0 * (headPVA.col(1) + tailPVA.col(1)) * t4_inv +
                                          6.0 * (tailPVA.col(0) - headPVA.col(0)) * t5_inv;
                coeffMatReversed.col(4) = (-tailPVA.col(2) + 1.5 * headPVA.col(2)) * t2_inv +
                                          (8.0 * headPVA.col(1) + 7.0 * tailPVA.col(1)) * t3_inv +
                                          15.0 * (-tailPVA.col(0) + headPVA.col(0)) * t4_inv;
                coeffMatReversed.col(3) = (0.5 * tailPVA.col(2) - 1.5 * headPVA.col(2)) * t1_inv -
                                          (6.0 * headPVA.col(1) + 4.0 * tailPVA.col(1)) * t2_inv +
                                          10.0 * (tailPVA.col(0) - headPVA.col(0)) * t3_inv;
                coeffMatReversed.col(2) = 0.5 * headPVA.col(2);//c2 = a0 / 2
                coeffMatReversed.col(1) = headPVA.col(1);//c1 = v0
                coeffMatReversed.col(0) = headPVA.col(0);//c0 = p0
                b = coeffMatReversed.transpose();
            }
            else//有中间点，多段轨迹
            {
                T1 = ts;
                T2 = T1.cwiseProduct(T1);
                T3 = T2.cwiseProduct(T1);
                T4 = T2.cwiseProduct(T2);
                T5 = T4.cwiseProduct(T1);

                A.reset();
                b.setZero();

                A(0, 0) = 1.0;
                A(1, 1) = 1.0;
                A(2, 2) = 2.0;//这三行约束第 0 段起点:p(0) = c0,v(0) = c1,a(0) = 2c2
                b.row(0) = headPVA.col(0).transpose();
                b.row(1) = headPVA.col(1).transpose();
                b.row(2) = headPVA.col(2).transpose();//右端项设置为起点的 PVA

                for (int i = 0; i < N - 1; i++)//中间段约束
                {
                    A(6 * i + 3, 6 * i + 3) = 6.0;
                    A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
                    A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
                    A(6 * i + 3, 6 * i + 9) = -6.0;//约束是6c3_i + 24T c4_i + 60T^2 c5_i - 6c3_{i+1} = 0
                    A(6 * i + 4, 6 * i + 4) = 24.0;
                    A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
                    A(6 * i + 4, 6 * i + 10) = -24.0;//24c4_i + 120T c5_i - 24c4_{i+1} = 0
                    A(6 * i + 5, 6 * i) = 1.0;
                    A(6 * i + 5, 6 * i + 1) = T1(i);
                    A(6 * i + 5, 6 * i + 2) = T2(i);
                    A(6 * i + 5, 6 * i + 3) = T3(i);
                    A(6 * i + 5, 6 * i + 4) = T4(i);
                    A(6 * i + 5, 6 * i + 5) = T5(i);
                    A(6 * i + 6, 6 * i) = 1.0;//第 i 段终点位置等于第 i+1 段起点位置,保证位置连续
                    A(6 * i + 6, 6 * i + 1) = T1(i);
                    A(6 * i + 6, 6 * i + 2) = T2(i);
                    A(6 * i + 6, 6 * i + 3) = T3(i);
                    A(6 * i + 6, 6 * i + 4) = T4(i);
                    A(6 * i + 6, 6 * i + 5) = T5(i);
                    A(6 * i + 6, 6 * i + 6) = -1.0;
                    A(6 * i + 7, 6 * i + 1) = 1.0;//速度连续
                    A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
                    A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
                    A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
                    A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
                    A(6 * i + 7, 6 * i + 7) = -1.0;
                    A(6 * i + 8, 6 * i + 2) = 2.0;//加速度连续
                    A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
                    A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
                    A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
                    A(6 * i + 8, 6 * i + 8) = -2.0;

                    b.row(6 * i + 5) = inPs.col(i).transpose();
                }

                A(6 * N - 3, 6 * N - 6) = 1.0;//终点位置约束
                A(6 * N - 3, 6 * N - 5) = T1(N - 1);
                A(6 * N - 3, 6 * N - 4) = T2(N - 1);
                A(6 * N - 3, 6 * N - 3) = T3(N - 1);
                A(6 * N - 3, 6 * N - 2) = T4(N - 1);
                A(6 * N - 3, 6 * N - 1) = T5(N - 1);
                A(6 * N - 2, 6 * N - 5) = 1.0;//终点速度约束
                A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
                A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
                A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
                A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
                A(6 * N - 1, 6 * N - 4) = 2;//终点加速度约束
                A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
                A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
                A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

                b.row(6 * N - 3) = tailPVA.col(0).transpose();
                b.row(6 * N - 2) = tailPVA.col(1).transpose();
                b.row(6 * N - 1) = tailPVA.col(2).transpose();//右端项对应终点 PVA

                A.factorizeLU();
                A.solve(b);//这两行真正解出所有多项式系数,b被覆盖成多项式系数

                return;
            }
        }

        inline const Eigen::MatrixXd &get_b() const//只读获取轨迹多项式系数
        {
            return b;
        }

        inline const Eigen::VectorXd &get_T1() const//只读获取每段时间
        {
            return T1;
        }

        inline Eigen::MatrixXd &get_gdC()//可修改地获取系数梯度矩阵
        {
            return gdC;
        }

        // inline Eigen::MatrixXd get_gdT() const
        // {
        //     return gdT;
        // }

        // inline Eigen::MatrixXd get_gdT(size_t i) const
        // {
        //     return gdT(i);
        // }

        inline double getTrajJerkCost() const//计算整条轨迹的 jerk 代价
        {
            double objective = 0.0;
            for (int i = 0; i < N; i++)
            {
                objective += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                             144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                             192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                             240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                             720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                             720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
            }
            return objective;
        }

        inline Trajectory getTraj(void) const//把优化器里的系数打包成 Trajectory
        {
            Trajectory traj;
            traj.reserve(N);
            for (int i = 0; i < N; i++)
            {
                traj.emplace_back(T1(i), b.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
            }
            return traj;
        }

        inline Eigen::MatrixXd getInitConstraintPoints(const int K) const//返回轨迹上的采样点矩阵
        {
            Eigen::MatrixXd pts(3, N * K + 1);
            Eigen::Vector3d pos;
            Eigen::Matrix<double, 6, 1> beta0;
            double s1, s2, s3, s4, s5;
            double step;
            int i_dp = 0;

            for (int i = 0; i < N; ++i)
            {
                const auto &c = b.block<6, 3>(i * 6, 0);
                step = T1(i) / K;
                s1 = 0.0;
                double t = 0;
                // innerLoop = K;

                for (int j = 0; j <= K; ++j)
                {
                    s2 = s1 * s1;
                    s3 = s2 * s1;
                    s4 = s2 * s2;
                    s5 = s4 * s1;
                    beta0 << 1.0, s1, s2, s3, s4, s5;
                    pos = c.transpose() * beta0;
                    pts.col(i_dp) = pos;

                    s1 += step;
                    if (j != K || (j == K && i == N - 1))
                    {
                        ++i_dp;
                    }
                }
            }

            return pts;
        }

        template <typename EIGENVEC, typename EIGENMAT>//把系数梯度转成时间和中间点梯度
        inline void getGrad2TP(EIGENVEC &gdT,
                               EIGENMAT &gdInPs)
        {
            solveAdjGradC(gdC);
            addPropCtoT(gdC, gdT);
            addPropCtoP(gdC, gdInPs);
        }

        template <typename EIGENVEC>
        inline void initGradCost(EIGENVEC &gdT,
                                 double &cost)//初始化 jerk cost 和基础梯度
        {
            // printf( "gdInPs=%d\n", gdInPs.size() );

            gdT.setZero();
            gdC.setZero();//清空时间梯度和系数梯度
            cost = getTrajJerkCost();//计算基础 jerk cost
            addGradJbyT(gdT);
            addGradJbyC(gdC);//分别计算 jerk cost 对时间和系数的直接梯度
        }

        template <typename EIGENVEC, typename EIGENMAT>//完整计算 cost 和梯度
        inline void evalTrajCostGrad(const Eigen::VectorXi &cons,
                                     const Eigen::VectorXi &idxHs,
                                     const std::vector<Eigen::MatrixXd> &cfgHs,
                                     const double &vmax,
                                     const double &amax,
                                     const Eigen::Vector3d &ci,
                                     double &cost,
                                     EIGENVEC &gdT,
                                     EIGENMAT &gdInPs)
        {
            gdT.setZero();
            gdInPs.setZero();
            gdC.setZero();//清空所有梯度

            cost = getTrajJerkCost();
            addGradJbyT(gdT);
            addGradJbyC(gdC);//先加入最小 jerk 代价和它的直接梯度
            //再加入约束惩罚,位置是否跑出安全走廊,速度是否超过 vmax,加速度是否超过 amax,并且同时更新 cost、gdT、gdC
            addTimeIntPenalty(cons, idxHs, cfgHs, vmax, amax, ci, cost, gdT, gdC);

            solveAdjGradC(gdC);
            addPropCtoT(gdC, gdT);
            addPropCtoP(gdC, gdInPs);//最后用伴随法把系数梯度传播回每段时间 gdT,中间点 gdInPs
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} //namespace poly_traj
