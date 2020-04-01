using System;
using System.Collections.Generic;
using System.Text;

namespace MathNet.Numerics.LinearAlgebra
{
	public static class VectorExtensions
	{
        /// <summary>
        /// Inserts the given vector into the given matrix at colum "index" and row 0
        /// </summary>
        /// <param name="MainMatrix">Matrix where the vector gets inserted</param>
        /// <param name="SubVector">Vector to be inserted</param>
        /// <param name="index">Column index in the MainMatrix</param>
        /// <returns>The MainMatrix with the Vector inserted at [0, ColumnIndex]</returns>
        public static Matrix<double> InsertAtIndex(Matrix<double> MainMatrix, Vector<double> SubVector, int index)
        {
            for (int Row = 0; Row < SubVector.Count; Row++)
            {
                MainMatrix[Row, index] = SubVector[Row];
            }
            return MainMatrix;
        }

        /// <summary>
        /// Inserts the given vector into the given vector at index
        /// </summary>
        /// <param name="MainVector">Vector where the SubVector gets inserted</param>
        /// <param name="SubVector">Vector to be inserted</param>
        /// <param name="index">Index in the MainVector</param>
        /// <returns>Vector with SubVector inserted at index</returns>
        public static Vector<double> InsertAtIndex(Vector<double> MainVector, Vector<double> SubVector, int index)
        {
            for (int Row = index; Row < SubVector.Count + index; Row++)
            {
                MainVector[Row] = SubVector[Row];
            }
            return MainVector;
        }
    }
}
