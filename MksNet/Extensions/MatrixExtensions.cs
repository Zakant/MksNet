using System;
using System.Collections.Generic;
using System.Text;

namespace MathNet.Numerics.LinearAlgebra
{
	public static class MatrixExtensions
	{
		/// <summary>
		/// Inserts the given matrix at the given index (diagonal)
		/// </summary>
		/// <param name="MainMatrix">Matrix where the submatrix gets inserted</param>
		/// <param name="SubMatrix">The matrix to be inserted</param>
		/// <param name="Index">Diagonal index</param>
		/// <returns>The matrix with the SubMatrix inserted</returns>
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

		/// <summary>
		/// Inserts the given matrix at the given column and row index
		/// </summary>
		/// <param name="MainMatrix"></param>
		/// <param name="SubMatrix"></param>
		/// <param name="ColumnIndex"></param>
		/// <param name="RowIndex"></param>
		/// <returns>The matrix with the SubMatrix inserted</returns>
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

		/// <summary>
		/// Inserts the given vector into the given matrix at colum "index" and row 0
		/// </summary>
		/// <param name="MainMatrix">Matrix where the vector gets inserted</param>
		/// <param name="SubVector">Vector to be inserted</param>
		/// <param name="index">Column index in the MainMatrix</param>
		/// <returns>The MainMatrix with the Vector inserted at [0, ColumnIndex]</returns>
		public static Matrix<double> InsertAtIndex(this Matrix<double> MainMatrix, Vector<double> SubVector, int index, bool InsertColumnWise = true)
		{
			if (InsertColumnWise)
			{
				for (int Row = 0; Row < SubVector.Count; Row++)
				{
					MainMatrix[Row, index] = SubVector[Row];
				}
			}
			else
			{
				for (int Row = 0; Row < SubVector.Count; Row++)
				{
					MainMatrix[index + Row, 0] = SubVector[Row];
				}
			}
			
			return MainMatrix;
		}

		public static Matrix<double> InsertAtIndex(this Matrix<double> MainMatrix, Vector<double> SubVector, int ColumnIndex, int RowIndex, bool InsertColumnWise = true)
		{
			if (InsertColumnWise)
			{
				for (int Row = RowIndex; Row < SubVector.Count + RowIndex; Row++)
				{
					MainMatrix[Row, ColumnIndex] = SubVector[Row - RowIndex];
				}
			}
			else
			{
				for (int Column = ColumnIndex; Column < SubVector.Count + ColumnIndex; Column++)
				{
					MainMatrix[RowIndex, Column] = SubVector[Column - ColumnIndex];
				}
			}
			return MainMatrix;
		}
	}
}
