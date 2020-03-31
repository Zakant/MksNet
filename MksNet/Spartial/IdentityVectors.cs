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
        public static Vector<double> X { get; } = CreateVector.Dense<double>(new double[] { 1, 0, 0 });

        /// <summary>
        /// Identity Vector in y-Direction
        /// </summary>
        /// <returns></returns>
        public static Vector<double> Y { get; } = CreateVector.Dense<double>(new double[] { 0, 1, 0});

        /// <summary>
        /// Identity Vector in z-Direction
        /// </summary>
        /// <returns></returns>
        public static Vector<double> Z { get; } = CreateVector.Dense<double>(new double[] { 0, 0, 1 });
    }
}
