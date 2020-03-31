using System;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using MksNet.Spartial;

namespace MksNet.Elements
{
	/// <summary>
	/// Represents on element of a multibody system.
	/// </summary>
	public sealed class Element
	{
		/// <summary>
		/// Id of the element.
		/// </summary>
		public int ElementId { get; internal set; }

		/// <summary>
		/// System the element belongs to.
		/// </summary>
		public MultibodySystem System { get; internal set; }

		/// <summary>
		/// Parent element.
		/// </summary>
		public Element Parent { get; internal set; }

		/// <summary>
		/// Mass of the element.
		/// </summary>
		public double Mass { get; internal set; }

		/// <summary>
		/// Inertia of the element with respect to the element coordinate system.
		/// </summary>
		public Matrix<double> Inertia { get; internal set; }

		/// <summary>
		/// Origin frame of the element.
		/// </summary>
		public Frame Origin { get; internal set; }

		/// <summary>
		/// All frames attached to this frame.
		/// </summary>
		public IReadOnlyCollection<Frame> Frames { get; internal set; }

		/// <summary>
		/// Frame at which the center of mass is.
		/// </summary>
		public Frame Cog { get; internal set; }

		/// <summary>
		/// Calculates and inserts the local jacobians of the element and its children into the global jacobian
		/// </summary>
		/// <param name="GlobalJacobian">The global jacobian</param>
		/// <param name="GlobalStateVector">The global state-vector</param>
		/// <param name="ParentMatrix"></param>
		/// <param name="ParentVector"></param>
		/// <param name="ParentRotationalJacobian"></param>
		/// <param name="ParentRotatinalMatrixProduct"></param>
		/// <returns></returns>
		public Matrix<double> GetElementJacobian(Matrix<double> GlobalJacobian, Vector<double> GlobalStateVector, Matrix<double> ParentMatrix, Vector<double> ParentVector, Matrix<double> ParentRotationalJacobian, Matrix<double> ParentRotatinalMatrixProduct)
		{
			Matrix<double> LocalTranslationalJacobian;
			Matrix<double> LocalRotationalJacobian;
			Vector<double> LocalStateVector = GetLocalStateVector(GlobalStateVector);
			Matrix<double> LocalRotationalMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
			Matrix<double> RotationalMatrixProduct = ParentRotatinalMatrixProduct * LocalRotationalMatrix;
			Matrix<double> NewParentMatrix = GetNewParentMatrix(ParentMatrix, LocalStateVector);
			Vector<double> NewParentVector = GetNewParentVector(ParentVector, LocalStateVector, ParentMatrix);
			Matrix<double> NewParentRotationMatrixProduct = ParentRotatinalMatrixProduct * LocalRotationalMatrix;

			LocalTranslationalJacobian = GetTranslationalJacobianMatrix(GlobalStateVector, ParentMatrix, ParentVector);
			LocalRotationalJacobian = GetRotationalJacobian(GlobalStateVector, ParentRotationalJacobian, ParentRotatinalMatrixProduct);

			GlobalJacobian = GlobalJacobian.InsertAtIndex(LocalTranslationalJacobian, 0, this.ElementId * 6);
			GlobalJacobian = GlobalJacobian.InsertAtIndex(LocalRotationalMatrix, 0, this.ElementId * 6 + 3);

			for (int ElementIndex = 0; ElementIndex < Children.Length; ElementIndex++)
			{
				GlobalJacobian = Children[ElementIndex].GetElementJacobian(GlobalJacobian, GlobalStateVector, NewParentMatrix, NewParentVector, LocalRotationalJacobian, NewParentRotationMatrixProduct);
			}
			return GlobalJacobian;
		}

		private Vector<double> GetLocalStateVector(Vector<double> GlobalStateVector)
		{
			return GlobalStateVector.SubVector(6 * (this.ElementId - 1), 6);
		}

		private Matrix<double> GetNewParentMatrix(Matrix<double> ParentMatrix, Vector<double> LocalStateVector)
		{
			int NumElements = System.NumberOfElements;
			int ElementIndex = 6 * (this.ElementId - 1) * 3;
			Matrix<double> LocalRotationalMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
			Matrix<double> LocalPartialDerivativeAlpha = Rotation.GetAlphaPartialDerivate(LocalStateVector[3]);
			Matrix<double> LocalPartialDerivativeBeta = Rotation.GetBetaPartialDerivate(LocalStateVector[4]);
			Matrix<double> LocalPartialDerivativeGamma = Rotation.GetGammaPartialDerivate(LocalStateVector[5]);
			Matrix<double> LocalMatrix = CreateMatrix.Dense<double>(NumElements * 6 * 3, NumElements * 6 * 3);
			for (int i = 0; i < ElementIndex; i = i + 3)
			{
				LocalMatrix = LocalMatrix.InsertAtIndex(CreateMatrix.DenseIdentity<double>(9), i = i + 9);
			}
			for (int i = ElementIndex + 3 * 3; i < LocalMatrix.ColumnCount; i = i + 3)
			{
				LocalMatrix = LocalMatrix.InsertAtIndex(LocalRotationalMatrix, i);
			}
			LocalMatrix = LocalMatrix.InsertAtIndex(LocalPartialDerivativeAlpha, ElementIndex);
			LocalMatrix = LocalMatrix.InsertAtIndex(LocalPartialDerivativeBeta, ElementIndex + 3);
			LocalMatrix = LocalMatrix.InsertAtIndex(LocalPartialDerivativeGamma, ElementIndex + 6);

			return ParentMatrix * LocalMatrix;
		}

		private Vector<double> GetNewParentVector(Vector<double> ParentVector, Vector<double> LocalStateVector, Matrix<double> ParentMatrix)
		{
			return ParentVector + ParentMatrix * LocalStateVector.SubVector(0, 3);
		}

		/// <summary>
		/// Computates the Translational Jacobian of the Element and all it's children
		/// </summary>
		/// <param name="stateVector">The global State Vector</param>
		/// <param name="ParentRotationMatrix">The Matrix containing all needed Matrix Products from the Parent</param>
		/// <param name="ParentVector">The Vector containing the needed vector-shifts from the Parent</param>
		/// <returns></returns>
		public Matrix<double> GetTranslationalJacobianMatrix(Vector<double> LocalStateVector, Matrix<double> ParentMatrix, Vector<double> ParentVector)
		{
			Vector<double> TranslationalJacobian;
			Matrix<double> LocalRotationMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
			Vector<double> COG = CreateVector.Dense<double>(new double[] { 1, 2, 3 }); /// HAS TO EE REMOVED AND REPLACED WITH THE ACTUAL COG VECTOR!! (I CURRENTLY DO NOT UNDERSTAND HOW THE FRAMES SHOULD BE USED)

			int numElements = System.NumberOfElements; /// SHOULD BE REPLACED WITH VALUE FROM SYSTEM
			Vector<double> LocalVector = CreateVector.Dense<double>(numElements * 6 * 3);
			int ElementIndex = 6 * (this.ElementId - 1) * 3;
			for (int i = 9; i < ElementIndex * 6 * 3; i = i + 3)
			{
				LocalVector = LocalVector.InsertAtIndex(COG, i);
			}

			TranslationalJacobian = ParentVector + ParentMatrix * (LocalStateVector.SubVector(0, 3) + LocalRotationMatrix * COG);
			return TranslationalJacobian.ToRowMatrix();
		}

		/// <summary>
		/// Calculates the rotational jacobian of the element
		/// </summary>
		/// <param name="stateVector">Global state vector</param>
		/// <param name="ParentRotationalJacobian">The jacobian of the previous element</param>
		/// <param name="ParentRotatinalMatrixProduct">The product of all rotational matrices of the previous elements</param>
		/// <returns></returns>
		public Matrix<double> GetRotationalJacobian(Vector<double> GlobalStateVector, Matrix<double> ParentRotationalJacobian, Matrix<double> ParentRotatinalMatrixProduct)
		{
			int numElements = GlobalStateVector.Count; /// SHOULD BE REPLACED WITH VALUE FROM SYSTEM
			Matrix<double> RotationalJacobian = CreateMatrix.Dense<double>(3, numElements * 6);
			int LocalStateIndex = 6 * (this.ElementId - 1);
			Vector<double> LocalStateVector = GlobalStateVector.SubVector(LocalStateIndex, 6);
			Vector<double> LocalTranslationalStates = LocalStateVector.SubVector(0, 3);
			Vector<double> LocalRotationalStates = LocalStateVector.SubVector(3, 3);
			Matrix<double> LocalRotationalMatrixAlpha = Rotation.GetX(LocalRotationalStates[0]);
			Matrix<double> LocalRotationalMatrixBeta = Rotation.GetY(LocalRotationalStates[1]);
			Matrix<double> LocalRotationalMatrixGamma = Rotation.GetZ(LocalRotationalStates[2]);

			Vector<double> PartialDerivativeAlphaDot = ParentRotatinalMatrixProduct * LocalRotationalMatrixGamma * LocalRotationalMatrixBeta * IdentityVectors.X;
			Vector<double> PartialDerivativeBetaDot = ParentRotatinalMatrixProduct * LocalRotationalMatrixGamma * IdentityVectors.Y;
			Vector<double> PartialDerivativeGammaDot = ParentRotatinalMatrixProduct * IdentityVectors.Z;

			RotationalJacobian = RotationalJacobian.InsertAtIndex(PartialDerivativeAlphaDot, LocalStateIndex + 3);
			RotationalJacobian = RotationalJacobian.InsertAtIndex(PartialDerivativeBetaDot, LocalStateIndex + 4);
			RotationalJacobian = RotationalJacobian.InsertAtIndex(PartialDerivativeGammaDot, LocalStateIndex + 5);

			RotationalJacobian = RotationalJacobian + ParentRotationalJacobian;

			return RotationalJacobian;
		}
	}
}