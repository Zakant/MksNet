using System;
using System.Collections.Generic;
using System.Text;

namespace MathNet.Numerics.LinearAlgebra
{
	public static class MatrixExtensions
	{
		public static Matrix<double> InsertAtIndex(this Matrix<double> MainMatrix, Matrix<double> SubMatrix, int Index)
		{
			for (int Column = Index; Column < Index + SubMatrix.ColumnCount; Column++)
			{
				for (int Row = Index; Row < Index + SubMatrix.RowCount; Row++)
				{
					MainMatrix[Row, Column] = SubMatrix[Row - Index, Column - Index];
				}
			}
			return MainMatrix;
		}

		public static Matrix<double> InsertAtIndex(this Matrix<double> MainMatrix, Matrix<double> SubMatrix, int ColumnIndex, int RowIndex)
		{
			for (int Column = ColumnIndex; Column < ColumnIndex + SubMatrix.ColumnCount; Column++)
			{
				for (int Row = RowIndex; Row < RowIndex + SubMatrix.RowCount; Row++)
				{
					MainMatrix[Row, Column] = SubMatrix[Row - RowIndex, Column - ColumnIndex];
				}
			}
			return MainMatrix;
		}

		public static Matrix<double> InsertAtIndex(this Matrix<double> MainMatrix, Vector<double> SubVector, int index)
		{
			for (int Row = 0; Row < SubVector.Count; Row++)
			{
				MainMatrix[Row, index] = SubVector[Row];
			}
			return MainMatrix;
		}
	}
}
