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
        public int ElementId { get; private set; }

        /// <summary>
        /// System the element belongs to.
        /// </summary>
        public MultibodySystem System { get; private set; }

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

            GlobalJacobian = InsertMatrixAtIndex(GlobalJacobian, LocalTranslationalJacobian, 0, this.ElementId * 6);
            GlobalJacobian = InsertMatrixAtIndex(GlobalJacobian, LocalRotationalMatrix, 0, this.ElementId * 6 + 3);
            
            for(int ElementIndex = 0; ElementIndex < Children.Length; ElementIndex++)
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
            Matrix<double> LocalMatrix = CreateMatrix.Dense<double>(NumElements*6*3, NumElements*6*3);
            for(int i = 0; i < ElementIndex; i = i + 3)
            {
                LocalMatrix = InsertMatrixAtIndex(LocalMatrix, CreateMatrix.DenseIdentity<double>(9), i = i + 9);
            }
            for(int i = ElementIndex + 3 * 3; i < LocalMatrix.ColumnCount; i = i + 3)
            {
                LocalMatrix = InsertMatrixAtIndex(LocalMatrix, LocalRotationalMatrix, i);
            }
            LocalMatrix = InsertMatrixAtIndex(LocalMatrix, LocalPartialDerivativeAlpha, ElementIndex);
            LocalMatrix = InsertMatrixAtIndex(LocalMatrix, LocalPartialDerivativeBeta, ElementIndex + 3);
            LocalMatrix = InsertMatrixAtIndex(LocalMatrix, LocalPartialDerivativeGamma, ElementIndex + 6);

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
        public Matrix<double>  GetTranslationalJacobianMatrix(Vector<double> LocalStateVector, Matrix<double> ParentMatrix, Vector<double> ParentVector)
        {
            Vector<double> TranslationalJacobian;
            Matrix<double> LocalRotationMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
            Vector<double> COG = CreateVector.Dense<double>(new double[] { 1, 2, 3 }); /// HAS TO EE REMOVED AND REPLACED WITH THE ACTUAL COG VECTOR!! (I CURRENTLY DO NOT UNDERSTAND HOW THE FRAMES SHOULD BE USED)
            
            int numElements = System.NumberOfElements; /// SHOULD BE REPLACED WITH VALUE FROM SYSTEM
            Vector<double> LocalVector = CreateVector.Dense<double>(numElements * 6 * 3);
            int ElementIndex = 6 * (this.ElementId - 1) * 3;
            for (int i = 9; i < ElementIndex * 6 * 3; i = i + 3)
            {
                LocalVector = InsertVectorAtIndex(LocalVector, COG, i);
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

            Vector<double> PartialDerivativeAlphaDot = ParentRotatinalMatrixProduct * LocalRotationalMatrixGamma * LocalRotationalMatrixBeta * IdentityVectors.xAxis();
            Vector<double> PartialDerivativeBetaDot = ParentRotatinalMatrixProduct * LocalRotationalMatrixGamma * IdentityVectors.yAxis();
            Vector<double> PartialDerivativeGammaDot = ParentRotatinalMatrixProduct * IdentityVectors.zAxis();

            RotationalJacobian = InsertVectorAtIndex(RotationalJacobian, PartialDerivativeAlphaDot, LocalStateIndex + 3);
            RotationalJacobian = InsertVectorAtIndex(RotationalJacobian, PartialDerivativeBetaDot, LocalStateIndex + 4);
            RotationalJacobian = InsertVectorAtIndex(RotationalJacobian, PartialDerivativeGammaDot, LocalStateIndex + 5);

            RotationalJacobian = RotationalJacobian + ParentRotationalJacobian;

            return RotationalJacobian;
        }

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

        private Matrix<double> InsertVectorAtIndex(Matrix<double> MainMatrix, Vector<double> SubVector, int index)
        {
            for(int Row = 0; Row < SubVector.Count; Row++)
            {
                MainMatrix[Row, index] = SubVector[Row];
            }
            return MainMatrix;
        }
        
        private Vector<double> InsertVectorAtIndex(Vector<double> MainVector, Vector<double> SubVector, int index)
        {
            for(int Row = index; Row < SubVector.Count + index; Row++)
            {
                MainVector[Row] = SubVector[Row];
            }
            return MainVector;
        }

        private Vector<double>
    }
}
