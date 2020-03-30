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
        public Element Parent { get; private set; }

        /// <summary>
        /// Child elements
        /// </summary>
        public Element[] Children { get; private set; }

        /// <summary>
        /// Origin Frame
        /// </summary>
        public Frame Origin { get; private set; }

        public Frame COG { get; private set; }
        /// <summary>
        /// Calculates and inserts the local jacobians of the element and its children into the global jacobian
        /// </summary>
        /// <param name="GlobalJacobian">The global jacobian</param>
        /// <param name="GlobalStateVector">The global state-vector</param>
        /// <param name="ParentMatrix"></param>
        /// <param name="ParentVector"></param>
        /// <param name="ParentRotationalJacobian">The rotational jacobian of the parent</param>
        /// <param name="ParentRotatinalMatrixProduct">The product of the parent element and its parents rotation matrices</param>
        /// <returns></returns>
        public Matrix<double> GetElementJacobian(Matrix<double> GlobalJacobian, Vector<double> GlobalStateVector, Matrix<double> ParentMatrix, Matrix<double> ParentVector, Matrix<double> ParentRotationalJacobian, Matrix<double> ParentRotatinalMatrixProduct)
        {
            Matrix<double> LocalTranslationalJacobian;
            Matrix<double> LocalRotationalJacobian;
            Vector<double> LocalStateVector = GetLocalStateVector(GlobalStateVector);
            Matrix<double> LocalRotationalMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
            Matrix<double> NewParentMatrix = GetNewParentMatrix(ParentMatrix, LocalStateVector);
            Matrix<double> LocalVectorCOG = GetLocalVectorMatrix(6 * (this.ElementId - 1) * 3, CreateVector.Dense<double>(3)); /// COG-Vector is missing!!!!
            Matrix<double> LocalVectorParent = GetLocalVectorMatrix(6 * (this.ElementId - 1) * 3, LocalStateVector.SubVector(0,3) + CreateVector.Dense<double>(3)); /// Vector from Parent-Joint to Child-Joint is Missing!!!!

            Matrix<double> NewParentVector = GetNewParentVector(ParentVector, ParentMatrix, LocalVectorParent);
            Matrix<double> NewParentRotationMatrixProduct = ParentRotatinalMatrixProduct * LocalRotationalMatrix;

            LocalTranslationalJacobian = GetTranslationalJacobianMatrix(LocalVectorCOG, NewParentMatrix, ParentVector);
            LocalRotationalJacobian = GetRotationalJacobian(GlobalStateVector, ParentRotationalJacobian, ParentRotatinalMatrixProduct);

            GlobalJacobian = InsertMatrixAtIndex(GlobalJacobian, LocalTranslationalJacobian, 0, this.ElementId * 6);
            GlobalJacobian = InsertMatrixAtIndex(GlobalJacobian, LocalRotationalMatrix, 0, this.ElementId * 6 + 3);

            for(int ElementIndex = 0; ElementIndex < Children.Length; ElementIndex++)
            {
                GlobalJacobian = Children[ElementIndex].GetElementJacobian(GlobalJacobian, GlobalStateVector, NewParentMatrix, NewParentVector, LocalRotationalJacobian, NewParentRotationMatrixProduct);
            }
            return GlobalJacobian;
        }

        /// <summary>
        /// Calculates the time derivative of the local jacobian matrix
        /// </summary>
        /// <param name="GlobalJacobianDerivative">Time derivative of the global jacobian</param>
        /// <param name="GlobalStateVector">The global state vector</param>
        /// <param name="ParentMatrix">The needed matrix from the parent element</param>
        /// <param name="ParentMatrixDerivative">Time derivative of the parent matrix</param>
        /// <param name="ParentVector">Needed vector from parent element</param>
        /// <param name="ParentVectorDerivative">Time derivative of parent vector</param>
        /// <param name="ParentRotationalJacobianDerivative">The time derivative of the parent local rotation jacobian</param>
        /// <param name="ParentRotationlMatrixProduct">Product of the rotation matrix from all parent elements</param>
        /// <param name="ParentRotationlMatrixProductDerivative">Time derivative of the product of the rotation matrix from all parent elements</param>
        /// <returns></returns>
        public Matrix<double> GetElementJacobianDerivative(Matrix<double> GlobalJacobianDerivative, Vector<double> GlobalStateVector, Matrix<double> ParentMatrix, Matrix<double> ParentMatrixDerivative, Matrix<double> ParentVector, Matrix<double> ParentVectorDerivative, Matrix<double> ParentRotationalJacobianDerivative, Matrix<double> ParentRotationlMatrixProduct, Matrix<double> ParentRotationlMatrixProductDerivative)
        {
            int LocalStateIndex = 6 * (this.ElementId - 1);
            Matrix<double> LocalTranslationalJacobianDerivative, LocalRotationalJacobianDerivative;
            Matrix<double> NewParentMatrix, NewParentMatrixDerivative, NewParentVector, NewParentVectorDerivative;
            Vector<double> LocalStateVector = GetLocalStateVector(GlobalStateVector);
            Matrix<double> LocalRotationMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
            Matrix<double> LocalRotationalMatrixTotalDerivative = Rotation.GetTotalTimeDerivative(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5], LocalStateVector[3 + 6], LocalStateVector[4 + 6], LocalStateVector[5 + 6]);

            Matrix<double> NewParentRotationalMatrixProduct = ParentRotationlMatrixProduct * LocalRotationMatrix;
            Matrix<double> NewParentRotationalMatrixProductDerivative = ParentRotationlMatrixProduct * LocalRotationalMatrixTotalDerivative + ParentRotationlMatrixProductDerivative * LocalRotationMatrix;

            LocalTranslationalJacobianDerivative = GetTranslationalJacobianDerivative();
            LocalRotationalJacobianDerivative = GetRotationalJacobianDerivative(LocalStateVector, ParentRotationalJacobianDerivative, ParentRotationlMatrixProduct, ParentRotationlMatrixProductDerivative, LocalStateIndex);

            GlobalJacobianDerivative = InsertMatrixAtIndex(GlobalJacobianDerivative, LocalTranslationalJacobianDerivative, 0, this.ElementId * 6);
            GlobalJacobianDerivative = InsertMatrixAtIndex(GlobalJacobianDerivative, LocalRotationalJacobianDerivative, 0, this.ElementId * 6 + 3);
            for (int ElementIndex = 0; ElementIndex < Children.Length; ElementIndex++)
            {
                GlobalJacobianDerivative = Children[ElementIndex].GetElementJacobianDerivative(GlobalJacobianDerivative, GlobalStateVector, NewParentMatrix, NewParentMatrixDerivative, NewParentVector, NewParentVectorDerivative, LocalRotationalJacobianDerivative, NewParentRotationalMatrixProduct, NewParentRotationalMatrixProductDerivative);
            }
            return GlobalJacobianDerivative;

        }

        /// <summary>
        /// Computates the Translational Jacobian of the Element and all it's children
        /// </summary>
        /// <param name="stateVector">The global State Vector</param>
        /// <param name="ParentRotationMatrix">The Matrix containing all needed Matrix Products from the Parent</param>
        /// <param name="ParentVector">The Vector containing the needed vector-shifts from the Parent</param>
        /// <returns></returns>
        public Matrix<double> GetTranslationalJacobianMatrix(Matrix<double> LocalVector, Matrix<double> ParentMatrix, Matrix<double> ParentVector)
        {
            return ParentVector + LocalVector.Transpose() * ParentMatrix.Transpose();
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
            Vector<double> LocalStateVector = GetLocalStateVector(GlobalStateVector);
            Vector<double> LocalTranslationalStates = LocalStateVector.SubVector(0, 3);
            Vector<double> LocalRotationalStates = LocalStateVector.SubVector(3, 3);
            Matrix<double> LocalRotationalMatrixAlpha = Rotation.GetX(LocalRotationalStates[0]);
            Matrix<double> LocalRotationalMatrixBeta = Rotation.GetY(LocalRotationalStates[1]);
            Matrix<double> LocalRotationalMatrixGamma = Rotation.GetZ(LocalRotationalStates[2]);

            Vector<double> PartialDerivativeAlphaDot = ParentRotatinalMatrixProduct * LocalRotationalMatrixGamma * LocalRotationalMatrixBeta * IdentityVectors.xAxis();
            Vector<double> PartialDerivativeBetaDot = ParentRotatinalMatrixProduct * LocalRotationalMatrixGamma * IdentityVectors.yAxis();
            Vector<double> PartialDerivativeGammaDot = ParentRotatinalMatrixProduct * IdentityVectors.zAxis();

            RotationalJacobian = InsertVectorAtIndex(RotationalJacobian, PartialDerivativeAlphaDot, LocalStateIndex + 3);
            RotationalJacobian = InsertVectorAtIndex(RotationalJacobian, PartialDerivativeBetaDot, LocalStateIndex + 4);
            RotationalJacobian = InsertVectorAtIndex(RotationalJacobian, PartialDerivativeGammaDot, LocalStateIndex + 5);

            RotationalJacobian = RotationalJacobian + ParentRotationalJacobian;

            return RotationalJacobian;
        }

        /// <summary>
        /// Calculates the derivative of the local translational jacobian
        /// </summary>
        /// <returns></returns>
        public Matrix<double> GetTranslationalJacobianDerivative()
        {
            throw NotImplementedException();
        }

        /// <summary>
        /// Calculates the derivative of the local rotational jacobian
        /// </summary>
        /// <param name="LocalStateVector">The state vector of the element states</param>
        /// <param name="ParentRotationalJacobianDerivative">Derivative of the rotational jacobian from the parent element</param>
        /// <param name="ParentRotationalMatrixProduct">Product of the rotation matrix from all parent elements</param>
        /// <param name="ParentRotationalMatrixProductDerivative">Derivative of the Product of the rotation matrix from all parent elements</param>
        /// <param name="ElementIndex">Index of the element</param>
        /// <returns></returns>
        public Matrix<double> GetRotationalJacobianDerivative(Vector<double> LocalStateVector, Matrix<double> ParentRotationalJacobianDerivative, Matrix<double> ParentRotationalMatrixProduct, Matrix<double> ParentRotationalMatrixProductDerivative, int ElementIndex)
        {
            Matrix<double> LocalRotationalJacobianDerivative = CreateMatrix.Dense<double>(3, System.NumberOfElements * 6);
            Matrix<double> LocalRotationMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
            Matrix<double> LocalRotationalMatrixTotalDerivative = Rotation.GetTotalTimeDerivative(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5], LocalStateVector[3 + 6], LocalStateVector[4 + 6], LocalStateVector[5 + 6]);
            Matrix<double> LocalPartialMatrixBeta = Rotation.GetBetaPartialDerivate(LocalStateVector[4]);
            Matrix<double> LocalPartialMatrixGamma = Rotation.GetGammaPartialDerivate(LocalStateVector[5]);
            Matrix<double> LocalPartialMatrixBetaTotalDerivative = Rotation.GetBetaTimeDerivativeOfPartial(LocalStateVector[4], LocalStateVector[4 + 6]);
            Matrix<double> LocalPartialMatrixGammaTotalDerivative = Rotation.GetGammaTimeDerivativeOfPartial(LocalStateVector[5], LocalStateVector[5 + 6]);

            Vector<double> DerivativeAlpha = (ParentRotationalMatrixProduct * (LocalPartialMatrixGammaTotalDerivative * LocalPartialMatrixBeta + LocalPartialMatrixGamma * LocalPartialMatrixBetaTotalDerivative) + ParentRotationalMatrixProductDerivative * LocalPartialMatrixGamma * LocalPartialMatrixBeta) * IdentityVectors.xAxis();
            Vector<double> DerivativeBeta = (ParentRotationalMatrixProduct * (LocalPartialMatrixGammaTotalDerivative) + ParentRotationalMatrixProductDerivative * LocalPartialMatrixGamma) * IdentityVectors.zAxis();
            Vector<double> DerivativeGamma = ParentRotationalMatrixProductDerivative * IdentityVectors.zAxis();

            LocalRotationalJacobianDerivative = InsertVectorAtIndex(LocalRotationalJacobianDerivative, DerivativeAlpha, ElementIndex + 3);
            LocalRotationalJacobianDerivative = InsertVectorAtIndex(LocalRotationalJacobianDerivative, DerivativeBeta, ElementIndex + 4);
            LocalRotationalJacobianDerivative = InsertVectorAtIndex(LocalRotationalJacobianDerivative, DerivativeGamma, ElementIndex + 5);

            return ParentRotationalJacobianDerivative + LocalRotationalJacobianDerivative;

        }

        /// <summary>
        /// Extract the states of the elements out of the state-vector
        /// </summary>
        /// <param name="GlobalStateVector"></param>
        /// <returns>The local State-Vector. First 6 elements are the positions, last 6 are the velocities</returns>
        private Vector<double> GetLocalStateVector(Vector<double> GlobalStateVector)
        {
            Vector<double> LocalStateVector = CreateVector.Dense<double>(6 * 2);
            LocalStateVector = InsertVectorAtIndex(LocalStateVector, GlobalStateVector.SubVector(6 * (this.ElementId - 1), 6), 0);
            LocalStateVector = InsertVectorAtIndex(LocalStateVector, GlobalStateVector.SubVector(6 * ((this.ElementId - 1) + System.NumberOfElements), 6), 6);
            return LocalStateVector;
        }

        /// <summary>
        /// Creates the local vector for the translational jacobian with the local COG-vector
        /// </summary>
        /// <param name="ElementIndex">Index of the element in the global vector</param>
        /// <param name="COGVector">Local vector from the parent-joint to the COG of the element</param>
        /// <returns>Matrix containing the COG-vectors and unity vectors</returns>
        private Matrix<double> GetLocalVectorMatrix(int ElementIndex, Vector<double> COGVector)
        {
            Matrix<double> LocalVectorMatrix = CreateMatrix.Dense<double>(3, 6 * this.System.NumberOfElements);

            /// Insert the COG-Vector in all angular DOF proceeding and including the current Element
            for (int i = 2; i < ElementIndex; i += 6)
            {
                for (int j = 0; j < 3; j++)
                {
                    LocalVectorMatrix = InsertVectorAtIndex(LocalVectorMatrix, COGVector, i + j);
                }
            }

            /// Insert the unit vectors to the index of the translational DOF's of the current Element
            LocalVectorMatrix = InsertMatrixAtIndex(LocalVectorMatrix, CreateMatrix.DenseIdentity<double>(3), ElementIndex, 0);

            return LocalVectorMatrix;
        }

        /// <summary>
        /// Calculate the new parent matrix by parentmatrix * local matrix, where the local matrix has the local rotation matrix on the diagonal except on the local dof's where the partial derivatives are
        /// </summary>
        /// <param name="ParentMatrix">The matrix of the parent element</param>
        /// <param name="LocalStateVector">Vector containing the local states</param>
        /// <returns>The new ParentMatrix, calculated py ParentMatrix * LocalMatrix</returns>
        private Matrix<double> GetNewParentMatrix(Matrix<double> ParentMatrix, Vector<double> LocalStateVector)
        {
            int NumElements = System.NumberOfElements;
            int ElementIndex = 6 * (this.ElementId - 1) * 3;
            Matrix<double> LocalRotationalMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
            Matrix<double> LocalPartialDerivativeAlpha = Rotation.GetAlphaPartialDerivate(LocalStateVector[3]);
            Matrix<double> LocalPartialDerivativeBeta = Rotation.GetBetaPartialDerivate(LocalStateVector[4]);
            Matrix<double> LocalPartialDerivativeGamma = Rotation.GetGammaPartialDerivate(LocalStateVector[5]);
            Matrix<double> LocalMatrix = CreateMatrix.Dense<double>(NumElements*6*3, NumElements*6*3);
            for(int i = 0; i < LocalMatrix.ColumnCount; i = i + 3)
            {
                LocalMatrix = InsertMatrixAtIndex(LocalMatrix, LocalRotationalMatrix, i);
            }
            LocalMatrix = InsertMatrixAtIndex(LocalMatrix, CreateMatrix.DenseIdentity<double>(9), ElementIndex);
            LocalMatrix = InsertMatrixAtIndex(LocalMatrix, LocalPartialDerivativeAlpha, ElementIndex + 3 * 3);
            LocalMatrix = InsertMatrixAtIndex(LocalMatrix, LocalPartialDerivativeBeta, ElementIndex + 3 * 3 + 3);
            LocalMatrix = InsertMatrixAtIndex(LocalMatrix, LocalPartialDerivativeGamma, ElementIndex + 3 * 3 + 9);

            return ParentMatrix * LocalMatrix;
        }

        /// <summary>
        /// Calculates the new ParentVector
        /// </summary>
        /// <param name="ParentVector">Vector from the parent-element</param>
        /// <param name="NewParentMatrix">The parentMatrix of the elements</param>
        /// <param name="LocalVector">The local vector containing the sum of </param>
        /// <returns></returns>
        private Matrix<double> GetNewParentVector(Matrix<double> ParentVector, Matrix<double> NewParentMatrix, Matrix<double> LocalVector)
        {
            return ParentVector + LocalVector.Transpose() * NewParentMatrix.Transpose();
        }

        /// <summary>
        /// Insert the given Matrix at [index, index]
        /// </summary>
        /// <param name="MainMatrix">Main matrix where the SubMatrix gets inserted</param>
        /// <param name="SubMatrix">Matrix to be inserted</param>
        /// <param name="Index">Diagonal index</param>
        /// <returns>The MainMatrix with the SubMatrix inserted at [index, index]</returns>
        private Matrix<double> InsertMatrixAtIndex(Matrix<double> MainMatrix, Matrix<double> SubMatrix, int Index)
        {
            for(int Column = Index; Column < Index + SubMatrix.ColumnCount; Column++)
            {
                for(int Row = Index; Row < Index + SubMatrix.RowCount; Row++)
                {
                    MainMatrix[Row, Column] = SubMatrix[Row - Index, Column - Index];
                }
            }
            return MainMatrix;
        }

        /// <summary>
        /// Inserts SubMatrix at the given index-pair
        /// </summary>
        /// <param name="MainMatrix">Main matrix where the SubMatrix gets inserted</param>
        /// <param name="SubMatrix">Matrix to be inserted</param>
        /// <param name="ColumnIndex">ColumnIndex in the MainMatrix</param>
        /// <param name="RowIndex">RowIndex in the MainMatrix</param>
        /// <returns>The MainMatrix with the SubMatrix inserted at [RowIndex, ColumnIndex]</returns>
        private Matrix<double> InsertMatrixAtIndex(Matrix<double> MainMatrix, Matrix<double> SubMatrix, int ColumnIndex, int RowIndex)
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
        private Matrix<double> InsertVectorAtIndex(Matrix<double> MainMatrix, Vector<double> SubVector, int index)
        {
            for(int Row = 0; Row < SubVector.Count; Row++)
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
        private Vector<double> InsertVectorAtIndex(Vector<double> MainVector, Vector<double> SubVector, int index)
        {
            for(int Row = index; Row < SubVector.Count + index; Row++)
            {
                MainVector[Row] = SubVector[Row];
            }
            return MainVector;
        }
    }
}
