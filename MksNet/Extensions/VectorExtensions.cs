using System;
using System.Collections.Generic;
using System.Text;

namespace MathNet.Numerics.LinearAlgebra
{
	public static class VectorExtensions
	{
         /// <summary>
        /// Inserts the given vector into the given vector at index
        /// </summary>
        /// <param name="MainVector">Vector where the SubVector gets inserted</param>
        /// <param name="SubVector">Vector to be inserted</param>
        /// <param name="index">Index in the MainVector</param>
        /// <returns>Vector with SubVector inserted at index</returns>
        public static Vector<double> InsertAtIndex(this Vector<double> MainVector, Vector<double> SubVector, int index)
        {
            for (int Row = index; Row < SubVector.Count + index; Row++)
            {
                MainVector[Row] = SubVector[Row - index];
            }
            return MainVector;
        }

        /// <summary>
        /// Adds the subvector to the main vector starting at index
        /// </summary>
        /// <param name="MainVector">The main vector</param>
        /// <param name="SubVector">The subvector</param>
        /// <param name="index">The starting index of the summation</param>
        /// <returns>The main vector with the subvector added</returns>
        public static Vector<double> AddAtIndex(this Vector<double> MainVector, Vector<double> SubVector, int index)
        {
            for (int Row = index; Row < SubVector.Count + index; Row ++)
            {
                MainVector[Row] += SubVector[Row - index];
            }
            return MainVector;
        }
    }
}
