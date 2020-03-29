using System;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;

namespace MksNet.Spartial
{
    static class IdentityVectors
    {
        /// <summary>
        /// Identity Vector in x-Direction
        /// </summary>
        /// <returns></returns>
        public static Vector<double> xAxis()
        {
            return CreateVector.Dense<double>(new double[] { 1,0,0});
        }

        /// <summary>
        /// Identity Vector in y-Direction
        /// </summary>
        /// <returns></returns>
        public static Vector<double> yAxis()
        {
            return CreateVector.Dense<double>(new double[] { 0, 1, 0});
        }

        /// <summary>
        /// Identity Vector in z-Direction
        /// </summary>
        /// <returns></returns>
        public static Vector<double> zAxis()
        {
            return CreateVector.Dense<double>(new double[] { 0, 0, 1});
        }
    }
}
