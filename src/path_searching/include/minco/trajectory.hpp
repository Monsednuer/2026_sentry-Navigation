/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)
    2D adaptation for ground-robot MINCO backend (第三步，报告5.5.3 思路二).
    Original: GCOPTER gcopter/trajectory.hpp；此处裁剪为 2D（空间维 3->2），
    移除 RootFinder 相关的 max-rate 根求解（地面底盘不需要）。

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef MINCO_TRAJECTORY_HPP
#define MINCO_TRAJECTORY_HPP

#include <Eigen/Eigen>

#include <cmath>
#include <vector>

namespace minco
{

    template <int D>
    class Piece
    {
    public:
        typedef Eigen::Matrix<double, 2, D + 1> CoefficientMat;

    private:
        double duration;
        CoefficientMat coeffMat;

    public:
        Piece() = default;

        Piece(double dur, const CoefficientMat &cMat)
            : duration(dur), coeffMat(cMat) {}

        inline int getDim() const
        {
            return 2;
        }

        inline int getDegree() const
        {
            return D;
        }

        inline double getDuration() const
        {
            return duration;
        }

        inline const CoefficientMat &getCoeffMat() const
        {
            return coeffMat;
        }

        inline Eigen::Vector2d getPos(const double &t) const
        {
            Eigen::Vector2d pos(0.0, 0.0);
            double tn = 1.0;
            for (int i = D; i >= 0; i--)
            {
                pos += tn * coeffMat.col(i);
                tn *= t;
            }
            return pos;
        }

        inline Eigen::Vector2d getVel(const double &t) const
        {
            Eigen::Vector2d vel(0.0, 0.0);
            double tn = 1.0;
            int n = 1;
            for (int i = D - 1; i >= 0; i--)
            {
                vel += n * tn * coeffMat.col(i);
                tn *= t;
                n++;
            }
            return vel;
        }

        inline Eigen::Vector2d getAcc(const double &t) const
        {
            Eigen::Vector2d acc(0.0, 0.0);
            double tn = 1.0;
            int m = 1;
            int n = 2;
            for (int i = D - 2; i >= 0; i--)
            {
                acc += m * n * tn * coeffMat.col(i);
                tn *= t;
                m++;
                n++;
            }
            return acc;
        }

        inline Eigen::Vector2d getJer(const double &t) const
        {
            Eigen::Vector2d jer(0.0, 0.0);
            double tn = 1.0;
            int l = 1;
            int m = 2;
            int n = 3;
            for (int i = D - 3; i >= 0; i--)
            {
                jer += l * m * n * tn * coeffMat.col(i);
                tn *= t;
                l++;
                m++;
                n++;
            }
            return jer;
        }

        inline Eigen::Vector2d getSnap(const double &t) const
        {
            Eigen::Vector2d snp(0.0, 0.0);
            double tn = 1.0;
            int k = 1;
            int l = 2;
            int m = 3;
            int n = 4;
            for (int i = D - 4; i >= 0; i--)
            {
                snp += k * l * m * n * tn * coeffMat.col(i);
                tn *= t;
                k++;
                l++;
                m++;
                n++;
            }
            return snp;
        }
    };

    template <int D>
    class Trajectory
    {
    private:
        typedef std::vector<Piece<D>> Pieces;
        Pieces pieces;

    public:
        Trajectory() = default;

        Trajectory(const std::vector<double> &durs,
                   const std::vector<typename Piece<D>::CoefficientMat> &cMats)
        {
            int N = std::min(durs.size(), cMats.size());
            pieces.reserve(N);
            for (int i = 0; i < N; i++)
            {
                pieces.emplace_back(durs[i], cMats[i]);
            }
        }

        inline int getPieceNum() const
        {
            return pieces.size();
        }

        inline Eigen::VectorXd getDurations() const
        {
            int N = getPieceNum();
            Eigen::VectorXd durations(N);
            for (int i = 0; i < N; i++)
            {
                durations(i) = pieces[i].getDuration();
            }
            return durations;
        }

        inline double getTotalDuration() const
        {
            int N = getPieceNum();
            double totalDuration = 0.0;
            for (int i = 0; i < N; i++)
            {
                totalDuration += pieces[i].getDuration();
            }
            return totalDuration;
        }

        inline Eigen::Matrix2Xd getPositions() const
        {
            int N = getPieceNum();
            Eigen::Matrix2Xd positions(2, N + 1);
            for (int i = 0; i < N; i++)
            {
                positions.col(i) = pieces[i].getCoeffMat().col(D);
            }
            positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
            return positions;
        }

        inline const Piece<D> &operator[](int i) const
        {
            return pieces[i];
        }

        inline Piece<D> &operator[](int i)
        {
            return pieces[i];
        }

        inline void clear(void)
        {
            pieces.clear();
            return;
        }

        inline bool empty(void) const
        {
            return pieces.empty();
        }

        inline typename Pieces::const_iterator begin() const
        {
            return pieces.begin();
        }

        inline typename Pieces::const_iterator end() const
        {
            return pieces.end();
        }

        inline typename Pieces::iterator begin()
        {
            return pieces.begin();
        }

        inline typename Pieces::iterator end()
        {
            return pieces.end();
        }

        inline void reserve(const int &n)
        {
            pieces.reserve(n);
            return;
        }

        inline void emplace_back(const Piece<D> &piece)
        {
            pieces.emplace_back(piece);
            return;
        }

        inline void emplace_back(const double &dur,
                                 const typename Piece<D>::CoefficientMat &cMat)
        {
            pieces.emplace_back(dur, cMat);
            return;
        }

        inline void append(const Trajectory<D> &traj)
        {
            pieces.insert(pieces.end(), traj.begin(), traj.end());
            return;
        }

        inline int locatePieceIdx(double &t) const
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
            return idx;
        }

        inline Eigen::Vector2d getPos(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getPos(t);
        }

        inline Eigen::Vector2d getVel(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getVel(t);
        }

        inline Eigen::Vector2d getAcc(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getAcc(t);
        }

        inline Eigen::Vector2d getJer(double t) const
        {
            int pieceIdx = locatePieceIdx(t);
            return pieces[pieceIdx].getJer(t);
        }

        inline Eigen::Vector2d getJuncPos(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(D);
            }
            else
            {
                return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::Vector2d getJuncVel(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(D - 1);
            }
            else
            {
                return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
            }
        }

        inline Eigen::Vector2d getJuncAcc(int juncIdx) const
        {
            if (juncIdx != getPieceNum())
            {
                return pieces[juncIdx].getCoeffMat().col(D - 2) * 2.0;
            }
            else
            {
                return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
            }
        }

        inline double getMaxVelRate() const
        {
            int N = getPieceNum();
            double maxVelRate = -INFINITY;
            const int samplesPerPiece = 16;
            for (int i = 0; i < N; i++)
            {
                const double dur = pieces[i].getDuration();
                for (int k = 0; k <= samplesPerPiece; k++)
                {
                    const double t = dur * k / samplesPerPiece;
                    const double vn = pieces[i].getVel(t).norm();
                    maxVelRate = maxVelRate < vn ? vn : maxVelRate;
                }
            }
            return maxVelRate;
        }

        inline double getMaxAccRate() const
        {
            int N = getPieceNum();
            double maxAccRate = -INFINITY;
            const int samplesPerPiece = 16;
            for (int i = 0; i < N; i++)
            {
                const double dur = pieces[i].getDuration();
                for (int k = 0; k <= samplesPerPiece; k++)
                {
                    const double t = dur * k / samplesPerPiece;
                    const double an = pieces[i].getAcc(t).norm();
                    maxAccRate = maxAccRate < an ? an : maxAccRate;
                }
            }
            return maxAccRate;
        }
    };

    typedef Trajectory<5> Trajectory5;

}

#endif
