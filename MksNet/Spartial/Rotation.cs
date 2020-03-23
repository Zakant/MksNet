using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Text;

namespace MksNet.Spartial
{
    /// <summary>
    /// Provides methods to obtain rotation matrices and derivatives with respect to angles.
    /// </summary>
    public static class Rotation
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetXYZ(double alpha, double beta, double gamma) => GetX(alpha) * GetY(beta) * GetZ(gamma);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetX(double alpha) => CreateMatrix.Dense(3, 3, new double[] { 1, 0, 0, 0, Math.Cos(alpha), Math.Sin(alpha), 0, -Math.Sin(alpha), Math.Cos(alpha) });

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetY(double beta) => CreateMatrix.Dense(3, 3, new double[] { Math.Cos(beta), 0, -Math.Sin(beta), 0, 1, 0, Math.Sin(beta), 0, Math.Cos(beta) });

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetZ(double gamma) => CreateMatrix.Dense(3, 3, new double[] { Math.Cos(gamma), Math.Sin(gamma), 0, -Math.Sin(gamma), Math.Cos(gamma), 0, 0, 1 });

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetAlphaPartialDerivate(double alpha)
        {
            return CreateMatrix.DenseOfArray(new double[,] { {0, 0, 0,}, {0, -Math.Sin(alpha), -Math.Cos(alpha)}, {0, Math.Cos(alpha), -Math.Sin(alpha)} });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetBetaPartialDerivate(double beta)
        {
            return CreateMatrix.DenseOfArray(new double[,] { {-Math.Sin(beta), 0, Math.Cos(beta)}, {0, 0, 0}, {-Math.Cos(beta), 0, -Math.Sin(beta)} });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetGammaPartialDerivate(double gamma)
        {
            return CreateMatrix.DenseOfArray(new double[,] { {-Math.Sin(gamma), -Math.Cos(gamma), 0}, {Math.Cos(gamma), -Math.Sin(gamma), 0}, {0, 0, 0} });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetAlphaTimeDerivativeOfPartial(double alpha, double dalpha_dt)
        {
            return CreateMatrix.DenseOfArray(new double[,] { {0, 0, 0}, {0, -Math.Cos(alpha), Math.Sin(alpha)}, {0, -Math.Sin(alpha), -Math.Cos(alpha)} }) * dalpha_dt;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetBetaTimeDerivativeOfPartial(double beta, double dbeta_dt)
        {
            return CreateMatrix.DenseOfArray(new double[,] { {-Math.Cos(beta), 0, -Math.Sin(beta)}, {0, 0, 0}, {Math.Sin(beta), 0, -Math.Cos(beta)} }) * dbeta_dt;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetGammaTimeDerivativeOfPartial(double gamma, double dgamma_dt)
        {
            return CreateMatrix.DenseOfArray(new double[,] { {-Math.Cos(gamma), Math.Sin(gamma), 0}, {-Math.Sin(gamma), -Math.Cos(gamma), 0}, {0, 0, 0} }) * dgamma_dt;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetAlphaTotalTimeDerivative(double alpha, double beta, double gamma, double dalpha_dt, double dbeta_dt, double dgamma_dt)
        {
            return GetGammaPartialDerivate(gamma) * dgamma_dt * GetY(beta) * GetAlphaPartialDerivate(alpha) + GetZ(gamma) * GetBetaPartialDerivate(beta) * dbeta_dt * GetAlphaPartialDerivate(alpha) + GetZ(gamma) * GetY(beta) * GetAlphaTimeDerivativeOfPartial(alpha, dalpha_dt);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetBetaTotalTimeDerivative(double alpha, double beta, double gamma, double dalpha_dt, double dbeta_dt, double dgamma_dt)
        {
            return GetGammaPartialDerivate(gamma) * dgamma_dt * GetBetaPartialDerivate(beta) * GetX(alpha) * GetZ(gamma) * GetBetaTimeDerivativeOfPartial(beta, dbeta_dt) * GetX(alpha) + GetZ(gamma) * GetBetaPartialDerivate(beta) * GetAlphaPartialDerivate(alpha) * dalpha_dt;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetGammaTotalTimeDerivative(double alpha, double beta, double gamma, double dalpha_dt, double dbeta_dt, double dgamma_dt)
        {
            return GetGammaTimeDerivativeOfPartial(gamma, dgamma_dt) * GetY(beta) * GetX(alpha) + GetGammaPartialDerivate(gamma) * GetBetaPartialDerivate(beta) * dbeta_dt * GetX(alpha) + GetGammaPartialDerivate(gamma) * GetY(beta) * GetAlphaPartialDerivate(alpha) * dalpha_dt;
        }
    }
}
