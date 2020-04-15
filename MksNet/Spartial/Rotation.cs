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
        /// <summary>
        /// Get the entire rotation matrix
        /// </summary>
        /// <param name="alpha"> Angle around the x-axis of the depending joint</param> 
        /// <param name="beta">  Angle around the y-axis of the depending joint</param>  
        /// <param name="gamma"> Angle around the z-axis of the depending joint</param> 
        /// <returns> The Rotation Matrix of the entire Joint </returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetXYZ(double alpha, double beta, double gamma)
        {
            return GetX(alpha) * GetY(beta) * GetZ(gamma);
        }

        public static Matrix<double> GetTotalTimeDerivative(double alpha, double beta, double gamma, double dalpha_dt, double dbeta_dt, double dgamma_dt)
        {
            return GetGammaPartialDerivate(gamma) * dgamma_dt * GetY(beta) * GetX(alpha) + GetZ(gamma) * GetBetaPartialDerivate(beta) * dbeta_dt * GetX(alpha) + GetZ(gamma) * GetY(beta) * GetAlphaPartialDerivate(alpha) * dalpha_dt;
        }

        /// <summary>
        /// Get the time derivative of the partial derivative of the entire rotation matrix whith respect to alpha DdS_dalpha_Dt
        /// </summary>
        /// <param name="alpha"></param>
        /// <param name="beta"></param>
        /// <param name="gamma"></param>
        /// <param name="dalpha_dt"></param>
        /// <param name="dbeta_dt"></param>
        /// <param name="dgamma_dt"></param>
        /// <returns>time derivative of the partial derivative of the entire rotation matrix whith respect to alpha</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetTotalTimeDerivativeToAlpha(double alpha, double beta, double gamma, double dalpha_dt, double dbeta_dt, double dgamma_dt)
        {
            return GetGammaPartialDerivate(gamma) * dgamma_dt * GetY(beta) * GetAlphaPartialDerivate(alpha) + GetZ(gamma) * GetBetaPartialDerivate(beta) * dbeta_dt * GetAlphaPartialDerivate(alpha) + GetZ(gamma) * GetY(beta) * GetAlphaTimeDerivativeOfPartial(alpha, dalpha_dt);
        }

        /// <summary>
        /// Get the time derivative of the partial derivative of the entire rotation matrix whith respect to beta DdS_dbeta_Dt
        /// </summary>
        /// <param name="alpha"></param>
        /// <param name="beta"></param>
        /// <param name="gamma"></param>
        /// <param name="dalpha_dt"></param>
        /// <param name="dbeta_dt"></param>
        /// <param name="dgamma_dt"></param>
        /// <returns> time derivative of the partial derivative of the entire rotation matrix whith respect to beta</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetTotalTimeDerivativeToBeta(double alpha, double beta, double gamma, double dalpha_dt, double dbeta_dt, double dgamma_dt)
        {
            return GetGammaPartialDerivate(gamma) * dgamma_dt * GetBetaPartialDerivate(beta) * GetX(alpha) * GetZ(gamma) * GetBetaTimeDerivativeOfPartial(beta, dbeta_dt) * GetX(alpha) + GetZ(gamma) * GetBetaPartialDerivate(beta) * GetAlphaPartialDerivate(alpha) * dalpha_dt;
        }

        /// <summary>
        /// Get the time derivative of the partial derivative of the entire rotation matrix whith respect to gamma DdS_dgamma_Dt
        /// </summary>
        /// <param name="alpha"></param>
        /// <param name="beta"></param>
        /// <param name="gamma"></param>
        /// <param name="dalpha_dt"></param>
        /// <param name="dbeta_dt"></param>
        /// <param name="dgamma_dt"></param>
        /// <returns> time derivative of the partial derivative of the entire rotation matrix whith respect to gamma</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetTotalTimeDerivativeToGamma(double alpha, double beta, double gamma, double dalpha_dt, double dbeta_dt, double dgamma_dt)
        {
            return GetGammaTimeDerivativeOfPartial(gamma, dgamma_dt) * GetY(beta) * GetX(alpha) + GetGammaPartialDerivate(gamma) * GetBetaPartialDerivate(beta) * dbeta_dt * GetX(alpha) + GetGammaPartialDerivate(gamma) * GetY(beta) * GetAlphaPartialDerivate(alpha) * dalpha_dt;
        }

        /// <summary>
        /// Get the rotation matrix around the x-axis
        /// </summary>
        /// <param name="alpha"> Angle around the x-axis</param>
        /// <returns>Rotation Matrix around the x-axis</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetX(double alpha) => CreateMatrix.Dense(3, 3, new double[] { 1, 0, 0, 0, Math.Cos(alpha), Math.Sin(alpha), 0, -Math.Sin(alpha), Math.Cos(alpha) });

        /// <summary>
        /// Get the rotation matrix around the y-axis
        /// </summary>
        /// <param name="beta"> Angle around the y-axis</param>
        /// <returns>Rotation Matrix around the y-axis</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetY(double beta) => CreateMatrix.Dense(3, 3, new double[] { Math.Cos(beta), 0, -Math.Sin(beta), 0, 1, 0, Math.Sin(beta), 0, Math.Cos(beta) });

        /// <summary>
        /// Get the rotation matrix around the z-axis
        /// </summary>
        /// <param name="gamma"> Angle around the z-axis</param>
        /// <returns>Rotation matrix around the z-axis</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetZ(double gamma) => CreateMatrix.Dense(3, 3, new double[] { Math.Cos(gamma), Math.Sin(gamma), 0, -Math.Sin(gamma), Math.Cos(gamma), 0, 0, 0, 1 });

        /// <summary>
        /// Get the partial derivative of the rotation matrix, with respect to alpha, around the x-axis
        /// </summary>
        /// <param name="alpha"> Angle around the x-axis</param>
        /// <returns>partial derivative of the rotation matrix, with respect to alpha, around the x-axis</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetAlphaPartialDerivate(double alpha)
        {
            return CreateMatrix.Dense(3, 3, new double[] { 0, 0, 0, 0, -Math.Sin(alpha), Math.Cos(alpha), 0, -Math.Cos(alpha), -Math.Sin(alpha)});
        }

        /// <summary>
        /// Get the partial derivative of the rotation matrix ,with respect to beta, around the y-axis
        /// </summary>
        /// <param name="beta"> Angle around the y-axis</param>
        /// <returns>partial derivative of the rotation matrix, with respect to beta, around the y-axis</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetBetaPartialDerivate(double beta)
        {
            return CreateMatrix.Dense(3, 3, new double[] {-Math.Sin(beta), 0, -Math.Cos(beta), 0, 0, 0, Math.Cos(beta), 0, -Math.Sin(beta)});
        }

        /// <summary>
        /// Get the partial derivative of the rotation matrix ,with respect to gamma, around the z-axis
        /// </summary>
        /// <param name="gamma"> Angle around the y-axis</param>
        /// <returns>partial derivative of the rotation matrix, with respect to gamma, around the z-axis</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetGammaPartialDerivate(double gamma)
        {
            return CreateMatrix.Dense(3, 3, new double[] {-Math.Sin(gamma), Math.Cos(gamma), 0, -Math.Cos(gamma), -Math.Sin(gamma), 0, 0, 0, 0});
        }

        /// <summary>
        /// Get the time derivative of the partial derivative of the rotation matrix around the x-axis with respect to alpha DdSalpha_dalpha_Dt
        /// </summary>
        /// <param name="alpha"></param>
        /// <param name="dalpha_dt"></param>
        /// <returns>time derivative of the partial derivative of the rotation matrix around the x-axis with respect to alpha</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetAlphaTimeDerivativeOfPartial(double alpha, double dalpha_dt)
        {
            return CreateMatrix.Dense(3, 3, new double[] {0, 0, 0, 0, -Math.Cos(alpha), -Math.Sin(alpha), 0, Math.Sin(alpha), -Math.Cos(alpha)}) * dalpha_dt;
        }

        /// <summary>
        /// Get the time derivative of the partial derivative of the rotation matrix around the y-axis with respect to beta DdSbeta_dbeta_Dt
        /// </summary>
        /// <param name="beta"></param>
        /// <param name="dbeta_dt"></param>
        /// <returns>time derivative of the partial derivative of the rotation matrix around the y-axis with respect to beta</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetBetaTimeDerivativeOfPartial(double beta, double dbeta_dt)
        {
            return CreateMatrix.Dense(3, 3, new double[] {-Math.Cos(beta), 0, Math.Sin(beta), 0, 0, 0, -Math.Sin(beta), 0, -Math.Cos(beta)}) * dbeta_dt;
        }

        /// <summary>
        /// Get the time derivative of the partial derivative of the rotation matrix around the z-axis with respect to gamma DdSgamma_dgamma_Dt
        /// </summary>
        /// <param name="gamma"></param>
        /// <param name="dgamma_dt"></param>
        /// <returns>time derivative of the partial derivative of the rotation matrix around the z-axis with respect to gamma</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Matrix<double> GetGammaTimeDerivativeOfPartial(double gamma, double dgamma_dt)
        {
            return CreateMatrix.Dense(3, 3, new double[] {-Math.Cos(gamma), -Math.Sin(gamma), 0, Math.Sin(gamma), -Math.Cos(gamma), 0, 0, 0, 0}) * dgamma_dt;
        }       
    }
}
