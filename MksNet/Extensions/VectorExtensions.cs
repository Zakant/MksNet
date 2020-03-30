using System;
using System.Collections.Generic;
using System.Text;

namespace MathNet.Numerics.LinearAlgebra
{
	public static class VectorExtensions
	{
        public static Vector<double> InsertAtIndex(this Vector<double> MainVector, Vector<double> SubVector, int index)
        {
            for (int Row = index; Row < SubVector.Count + index; Row++)
            {
                MainVector[Row] = SubVector[Row];
            }
            return MainVector;
        }
    }
}
