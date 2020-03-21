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
        public static Matrix<double> GetX(double alpha) => CreateMatrix.DenseOfArray(new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } });

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetY(double beta) => CreateMatrix.DenseOfArray(new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } });

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetZ(double gamma) => CreateMatrix.DenseOfArray(new double[,] { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } });

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetAlphaDerivate(double alpha, double beta, double gamma)
        {
            return CreateMatrix.DenseOfArray(new double[,] { { }, { }, { } });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetBetaDerivate(double alpha, double beta, double gamma)
        {
            return CreateMatrix.DenseOfArray(new double[,] { { }, { }, { } });
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetGammaDerivate(double alpha, double beta, double gamma)
        {
            return CreateMatrix.DenseOfArray(new double[,] { { }, { }, { } });
        }
    }
}
