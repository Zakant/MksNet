﻿using System;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using MksNet.Mbs.Joints;
using MksNet.Spartial;

namespace MksNet.Mbs.Elements
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
        /// Base joint of the element. Connects this elements Dertermines degrees of freedom.
        /// </summary>
        public Joint BaseJoint { get; internal set; }

        /// <summary>
        /// Child elements.
        /// </summary>
        public IReadOnlyCollection<Element> Children { get; internal set; }

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
        /// The matrix that transforms between all states and active states
        /// </summary>
        public Matrix<double> KeepMatrixScalar { get; internal set; }

        public Matrix<double> KeepMatrixIdentity { get; internal set; }

        /// <summary>
        /// The local P matrix (Local rotation along the Diagonal with partial differentials on the rotational states)
        /// </summary>
        public Matrix<double> LocalPMatrix { get; internal set; }

        public Matrix<double> LocalPVector { get; internal set; }

        /// <summary>
        /// Rotation matrix of the element
        /// </summary>
        public Matrix<double> LocalRotationMatrix { get; internal set; }

        /// <summary>
        /// The partial differential of the local rotation matrix with respect to the rotation along the x-axis
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartialDiffAlpha { get; internal set; }

        /// <summary>
        /// The partial differential of the local rotation matrix with respect to the rotation along the y-axis
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartialDiffBeta { get; internal set; }

        /// <summary>
        /// The partial differential of the local rotation matrix with respect to the rotation along the z-axis
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartialDiffGamma { get; internal set; }

        /// <summary>
        /// The total time derivative of the partial differential of the local rotation matrix with respect to the rotation along the x-axis
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartialDiffAlphaTotal { get; internal set; }

        /// <summary>
        /// The total time derivative of the partial differential of the local rotation matrix with respect to the rotation along the x-axis
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartialDiffBetaTotal { get; internal set; }

        /// <summary>
        /// The total time derivative of the partial differential of the local rotation matrix with respect to the rotation along the x-axis
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartialDiffGammaTotal { get; internal set; }

        /// <summary>
        /// The total time derivative of the local rotation matrix
        /// </summary>
        public Matrix<double> LocalRotationMatrixTotal { get; internal set; }


        /// <summary>
        /// Create the transformation matrices for keeping the necessary local calculations. One for scalars, one for matrices
        /// </summary>
        /// <param name="ActiveStates">Vector indicating the active states</param>
        public void CreateKeepMatric(Vector<double> ActiveStates)
        {
            int ActiveStatesSum = Convert.ToInt32(ActiveStates.Sum());
            this.KeepMatrixScalar = CreateMatrix.Dense<double>(ActiveStates.Count, ActiveStatesSum);
            for (int i = 0; i < ActiveStatesSum; i++)
            {
                this.KeepMatrixScalar.InsertAtIndex(ActiveStates, i);
            }
            this.KeepMatrixIdentity = CreateMatrix.Dense<double>(ActiveStates.Count * 3, ActiveStatesSum * 3);
            for(int i = 0; i < ActiveStatesSum * 3; i += 3)
            {
                this.KeepMatrixIdentity.InsertAtIndex(CreateMatrix.DenseIdentity<double>(3), i);
            }
        }

        /// <summary>
        /// Update the element with the state vector given in <paramref name="LocalStateVector"/>.
        /// </summary>
        /// <param name="LocalStateVector">Statevector used for update.</param>
        public void Update(Vector<double> LocalStateVector)
        {
            UpdatePMatrix(LocalStateVector);
            UpdatePVector(LocalStateVector);
            UpdateLocalRotationMatrices(LocalStateVector);
        }

        /// <summary>
        /// Updates the local P-Matrix
        /// </summary>
        /// <param name="LocalStateVector">Vector with the local states</param>
        private void UpdatePMatrix(Vector<double> LocalStateVector)
        {
            this.LocalPMatrix = GetLocalMatrix(LocalStateVector);
        }

        /// <summary>
        /// Updats the local P-Vector (without transformation
        /// </summary>
        /// <param name="LocalStateVector">Vector with local states</param>
        private void UpdatePVector(Vector<double> LocalStateVector)
        {
            this.LocalPVector = GetLocalVectorMatrix(GetElementIndex(), LocalStateVector); /// Vector from Parent origin to Local origin is missing
        }

        /// <summary>
        /// Updates all needed rotational matrices of the element
        /// </summary>
        /// <param name="LocalStateVector">Vector with local states</param>
        private void UpdateLocalRotationMatrices(Vector<double> LocalStateVector)
        {
            this.LocalRotationMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
            this.LocalRotationMatrixPartialDiffAlpha = Rotation.GetAlphaPartialDerivate(LocalStateVector[3]);
            this.LocalRotationMatrixPartialDiffBeta = Rotation.GetBetaPartialDerivate(LocalStateVector[4]);
            this.LocalRotationMatrixPartialDiffGamma = Rotation.GetGammaPartialDerivate(LocalStateVector[5]);
            this.LocalRotationMatrixPartialDiffAlphaTotal = Rotation.GetAlphaTimeDerivativeOfPartial(LocalStateVector[3], LocalStateVector[9]);
            this.LocalRotationMatrixPartialDiffBetaTotal = Rotation.GetBetaTimeDerivativeOfPartial(LocalStateVector[3], LocalStateVector[10]);
            this.LocalRotationMatrixPartialDiffGammaTotal = Rotation.GetGammaTimeDerivativeOfPartial(LocalStateVector[5], LocalStateVector[11]);
            this.LocalRotationMatrixTotal = Rotation.GetTotalTimeDerivative(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5], LocalStateVector[9], LocalStateVector[10], LocalStateVector[11]);
        }

        /// <summary>
        /// Calculates the product of all previous parent matrices
        /// </summary>
        /// <returns>The product of all parent matrices</returns>
        public Matrix<double> GetParentMatrix()
        {
            return Parent.GetParentMatrix() * this.LocalPMatrix;
        }

        /// <summary>
        /// Gets the parent vector
        /// </summary>
        /// <returns>The parent vector</returns>
        public Matrix<double> GetParentVector()
        {
            return Parent.GetParentVector() + this.LocalPVector.Transpose() * GetParentMatrix().Transpose();
        }

        /// <summary>
        /// Calculates the product of all parent rotation matrices
        /// </summary>
        /// <returns>The matrix product</returns>
        public Matrix<double> GetParentMatrixProdcut()
        {
            return Parent.GetParentMatrixProdcut() * this.LocalRotationMatrix;
        }

        /// <summary>
        /// Inserts the local mass matrix into the global one
        /// </summary>
        /// <param name="GlobalMassMatrix">The global mass matrix</param>
        /// <returns>Mass matrix with the local mass matrix of itsself and all children</returns>
        public Matrix<double> GetGlobalMassMatrix(Matrix<double> GlobalMassMatrix)
        {
            int ElementIndex = GetElementIndex();
            GlobalMassMatrix.InsertAtIndex(GetLocalMassMatrix(), ElementIndex);
            
            foreach(var child in Children)
            {
                child.GetGlobalMassMatrix(GlobalMassMatrix);
            }
            return GlobalMassMatrix;
        }

        /// <summary>
        /// Calculates the local mass matrix
        /// </summary>
        /// <returns>The local mass matrix</returns>
        private Matrix<double> GetLocalMassMatrix()
        {
            Matrix<double> LocalMassMatrix = CreateMatrix.Dense<double>(6, 6);
            LocalMassMatrix.InsertAtIndex(CreateMatrix.DenseIdentity<double>(3) * this.Mass, 0);
            LocalMassMatrix.InsertAtIndex(this.Inertia, 3);
            LocalMassMatrix = this.KeepMatrixScalar.Transpose() * LocalMassMatrix * this.KeepMatrixScalar;
            return LocalMassMatrix;
        }

        /// <summary>
        /// Inserts the local vector of forces into the global one, and adds its negative to the parent entries
        /// </summary>
        /// <param name="GlobalForceVector">The global force vector</param>
        /// <returns>The global force vector with the local forces of the element and its children inserted</returns>
        public Vector<double> GetGlobalForceMomentVector(Vector<double> GlobalForceVector, int ParentIndex)
        {
            int ElementIndex = GetElementIndex();
            Vector<double> LocalVector = GetLocalForceMomentVector();
            GlobalForceVector.InsertAtIndex(LocalVector, ElementIndex);
            GlobalForceVector.AddAtIndex(-LocalVector, ParentIndex);
            
            return GlobalForceVector;
        }

        /// <summary>
        /// Calculates the local force vector of the element
        /// </summary>
        /// <returns>The local force vector</returns>
        private Vector<double> GetLocalForceMomentVector()
        {
            Vector<double> LocalForceMomentVector = CreateVector.Dense<double>(6);
            Vector<double> LocalForceVector = CreateVector.Dense<double>(3);
            Vector<double> LocalMomentVector = CreateVector.Dense<double>(3);
            LocalForceVector += System.GravitationVector * this.Mass;
            LocalForceMomentVector.InsertAtIndex(LocalForceVector, 0);
            LocalForceMomentVector.InsertAtIndex(LocalMomentVector, 3);
            return this.KeepMatrixScalar * LocalForceMomentVector;
        }

        /// <summary>
        /// Inserts the local angular velocity into the global angular velocity vector
        /// </summary>
        /// <param name="GlobalAngularVelocity">The global angular velocity</param>
        /// <param name="GlobalStateVector">The global state vector</param>
        /// <param name="ParentRotationMatrixProduct">The product of the rotation matrix from all previous elements</param>
        /// <returns>The global vector of angular velocity with its own and all child elements local angular velocities</returns>
        public Vector<double> GetGlobalAngularVelocity(Vector<double> GlobalAngularVelocity, Vector<double> LocalStateVector, int GlobalIndex)
        {
            Vector<double> LocalAngularVelocity = GetLocalAngularVelocity(LocalStateVector);
            GlobalAngularVelocity.InsertAtIndex(LocalAngularVelocity, GlobalIndex + 3);
            return GlobalAngularVelocity;
        }

        /// <summary>
        /// Calculates the angular velocity of the element in the world frame
        /// </summary>
        /// <param name="GlobalStateVector">The global state vector</param>
        /// <returns>Vector of the transformed angular velocity</returns>
        public Vector<double> GetLocalAngularVelocity(Vector<double> LocalAngularVelocities)
        {

            return GetParentMatrixProdcut() * LocalAngularVelocities;
        }

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
        public Matrix<double> GetElementJacobian(Matrix<double> GlobalJacobian, Vector<double> GlobalStateVector, Matrix<double> ParentMatrix, Matrix<double> ParentVector, Matrix<double> ParentRotationalJacobian, Matrix<double> ParentRotatinalMatrixProduct)
        {
            int ElementIndex = GetElementIndex() * 3;
            Matrix<double> LocalTranslationalJacobian, LocalRotationalJacobian;

            Vector<double> LocalStateVector = GetLocalStateVector(GlobalStateVector);
            Matrix<double> LocalRotationalMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);

            Matrix<double> NewParentMatrix = GetNewParentMatrix(ParentMatrix, LocalStateVector);
            Matrix<double> LocalVectorCOG = GetLocalVectorMatrix(ElementIndex, Cog.Offset);
            LocalVectorCOG.InsertAtIndex(CreateMatrix.DenseIdentity<double>(3), ElementIndex, 0); /// Insert the unit vectors to the index of the translational DOF's of the current Element

            LocalTranslationalJacobian = GetTranslationalJacobianMatrix(LocalVectorCOG, NewParentMatrix, ParentVector);
            LocalRotationalJacobian = GetRotationalJacobian(GlobalStateVector, ParentRotationalJacobian, ParentRotatinalMatrixProduct);

            Matrix<double> LocalVectorState = GetLocalVectorMatrix(6 * (this.ElementId - 1) * 3, LocalStateVector.SubVector(0, 3) + CreateVector.Dense<double>(3)); /// Vector from Parent-Joint to Child-Joint is Missing!!!!
            LocalVectorState.InsertAtIndex(CreateMatrix.DenseIdentity<double>(3), ElementIndex, 0);

            Matrix<double> NewParentVector = GetNewParentVector(ParentVector, ParentMatrix, LocalVectorState);
            Matrix<double> NewParentRotationMatrixProduct = ParentRotatinalMatrixProduct * LocalRotationalMatrix;

            GlobalJacobian.InsertAtIndex(LocalTranslationalJacobian, 0, this.ElementId * 6);
            GlobalJacobian.InsertAtIndex(LocalRotationalMatrix, 0, this.ElementId * 6 + 3);

            foreach (var child in Children)
            {
                GlobalJacobian = child.GetElementJacobian(GlobalJacobian, GlobalStateVector, NewParentMatrix, NewParentVector, LocalRotationalJacobian, NewParentRotationMatrixProduct);
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
        public Matrix<double> GetElementJacobianDerivative(Matrix<double> GlobalJacobianDerivative, Vector<double> GlobalStateVector, Matrix<double> ParentMatrix, Matrix<double> ParentMatrixDerivative, Matrix<double> ParentVectorDerivative, Matrix<double> ParentRotationalJacobianDerivative, Matrix<double> ParentRotationlMatrixProduct, Matrix<double> ParentRotationlMatrixProductDerivative)
        {
            int GlobalIndex = GetElementIndex() * 3;
            Matrix<double> LocalTranslationalJacobianDerivative, LocalRotationalJacobianDerivative;
            Matrix<double> NewParentMatrix, NewParentMatrixDerivative, NewParentVectorDerivative;

            Vector<double> LocalStateVector = GetLocalStateVector(GlobalStateVector);
            Matrix<double> LocalRotationMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
            Matrix<double> LocalRotationalMatrixTotalDerivative = Rotation.GetTotalTimeDerivative(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5], LocalStateVector[3 + 6], LocalStateVector[4 + 6], LocalStateVector[5 + 6]);

            Matrix<double> LocalMatrix = GetLocalMatrix(LocalStateVector);
            Matrix<double> LocalMatrixDerivative = GetLocalMatrixDerivative(LocalStateVector);

            NewParentMatrix = ParentMatrix * LocalMatrix;
            NewParentMatrixDerivative = ParentMatrixDerivative * LocalMatrix + ParentMatrix * LocalMatrixDerivative;

            Matrix<double> LocalVectorCOG = GetLocalVectorMatrix(6 * (this.ElementId - 1) * 3, Cog.Offset);
            Matrix<double> LocalVectorCOGDerivative = CreateMatrix.Dense<double>(LocalVectorCOG.RowCount, LocalVectorCOG.ColumnCount);
            

            LocalTranslationalJacobianDerivative = GetTranslationalJacobianDerivative(NewParentMatrix, NewParentMatrixDerivative, ParentVectorDerivative, LocalVectorCOG, LocalVectorCOGDerivative);

            Matrix<double> NewParentRotationalMatrixProduct = ParentRotationlMatrixProduct * LocalRotationMatrix;
            Matrix<double> NewParentRotationalMatrixProductDerivative = ParentRotationlMatrixProduct * LocalRotationalMatrixTotalDerivative + ParentRotationlMatrixProductDerivative * LocalRotationMatrix;

            LocalRotationalJacobianDerivative = GetRotationalJacobianDerivative(LocalStateVector, ParentRotationalJacobianDerivative, ParentRotationlMatrixProduct, ParentRotationlMatrixProductDerivative, GlobalIndex);

            GlobalJacobianDerivative.InsertAtIndex(LocalTranslationalJacobianDerivative, 0, GlobalIndex);
            GlobalJacobianDerivative.InsertAtIndex(LocalRotationalJacobianDerivative, 0, GlobalIndex + 3);

            Matrix<double> LocalParentVector = GetLocalVectorMatrix(6 * (this.ElementId - 1) * 3, LocalStateVector.SubVector(0, 3) + CreateVector.Dense<double>(3));
            Matrix<double> LocalParentVectorDerivative = GetLocalVectorMatrix(6 * (this.ElementId - 1) * 3, LocalStateVector.SubVector(0, 3 + 6));

            NewParentVectorDerivative = ParentVectorDerivative + NewParentMatrixDerivative * LocalParentVector + NewParentMatrix * LocalParentVectorDerivative; /// Vector from Parent-Joint to Child-Joint is Missing!!!!

            foreach (var child in Children)
            {
                GlobalJacobianDerivative = child.GetElementJacobianDerivative(GlobalJacobianDerivative, GlobalStateVector, NewParentMatrix, NewParentMatrixDerivative, NewParentVectorDerivative, LocalRotationalJacobianDerivative, NewParentRotationalMatrixProduct, NewParentRotationalMatrixProductDerivative);
            }
            return GlobalJacobianDerivative;

        }

        /// <summary>
        /// Computates the Translational Jacobian of the Element
        /// </summary>
        /// <param name="LocalVector"></param>
        /// <param name="ParentMatrix"></param>
        /// <param name="ParentVector"></param>
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
            int numElements = this.System.NumberOfElements;
            Matrix<double> LocalRotationalJacobian = CreateMatrix.Dense<double>(3, numElements * 6);
            int LocalStateIndex = 6 * (this.ElementId - 1);
            Vector<double> LocalStateVector = GetLocalStateVector(GlobalStateVector);
            Vector<double> LocalRotationalStates = LocalStateVector.SubVector(3, 3);
            Matrix<double> LocalRotationalMatrixBeta = Rotation.GetY(LocalRotationalStates[1]);
            Matrix<double> LocalRotationalMatrixGamma = Rotation.GetZ(LocalRotationalStates[2]);

            Vector<double> PartialDerivativeAlphaDot = ParentRotatinalMatrixProduct * LocalRotationalMatrixGamma * LocalRotationalMatrixBeta * IdentityVectors.X;
            Vector<double> PartialDerivativeBetaDot = ParentRotatinalMatrixProduct * LocalRotationalMatrixGamma * IdentityVectors.Y;
            Vector<double> PartialDerivativeGammaDot = ParentRotatinalMatrixProduct * IdentityVectors.Z;

            LocalRotationalJacobian.InsertAtIndex(PartialDerivativeAlphaDot, LocalStateIndex + 3);
            LocalRotationalJacobian.InsertAtIndex(PartialDerivativeBetaDot, LocalStateIndex + 4);
            LocalRotationalJacobian.InsertAtIndex(PartialDerivativeGammaDot, LocalStateIndex + 5);

            LocalRotationalJacobian += ParentRotationalJacobian;

            return LocalRotationalJacobian;
        }

        /// <summary>
        /// Calculates the derivative of the local translational jacobian
        /// </summary>
        /// <returns></returns>
        public Matrix<double> GetTranslationalJacobianDerivative(Matrix<double> ParentMatrix, Matrix<double> ParentMatrixDerivative, Matrix<double> ParentVectorDerivative, Matrix<double> LocalVector, Matrix<double> LocalVectorDerivative)
        {
            return ParentVectorDerivative + LocalVector.Transpose() * ParentMatrixDerivative.Transpose() + LocalVectorDerivative.Transpose() * ParentMatrix.Transpose();
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
            Matrix<double> LocalPartialMatrixBeta = Rotation.GetBetaPartialDerivate(LocalStateVector[4]);
            Matrix<double> LocalPartialMatrixGamma = Rotation.GetGammaPartialDerivate(LocalStateVector[5]);
            Matrix<double> LocalPartialMatrixBetaTotalDerivative = Rotation.GetBetaTimeDerivativeOfPartial(LocalStateVector[4], LocalStateVector[4 + 6]);
            Matrix<double> LocalPartialMatrixGammaTotalDerivative = Rotation.GetGammaTimeDerivativeOfPartial(LocalStateVector[5], LocalStateVector[5 + 6]);

            Vector<double> DerivativeAlpha = (ParentRotationalMatrixProduct * (LocalPartialMatrixGammaTotalDerivative * LocalPartialMatrixBeta + LocalPartialMatrixGamma * LocalPartialMatrixBetaTotalDerivative) + ParentRotationalMatrixProductDerivative * LocalPartialMatrixGamma * LocalPartialMatrixBeta) * IdentityVectors.X;
            Vector<double> DerivativeBeta = (ParentRotationalMatrixProduct * (LocalPartialMatrixGammaTotalDerivative) + ParentRotationalMatrixProductDerivative * LocalPartialMatrixGamma) * IdentityVectors.Z;
            Vector<double> DerivativeGamma = ParentRotationalMatrixProductDerivative * IdentityVectors.Z;

            LocalRotationalJacobianDerivative.InsertAtIndex(DerivativeAlpha, ElementIndex + 3);
            LocalRotationalJacobianDerivative.InsertAtIndex(DerivativeBeta, ElementIndex + 4);
            LocalRotationalJacobianDerivative.InsertAtIndex(DerivativeGamma, ElementIndex + 5);

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
            LocalStateVector.InsertAtIndex(GlobalStateVector.SubVector(6 * (this.ElementId - 1), 6), 0);
            LocalStateVector.InsertAtIndex(GlobalStateVector.SubVector(6 * ((this.ElementId - 1) + System.NumberOfElements), 6), 6);
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
            Matrix<double> LocalVectorMatrix = CreateMatrix.Dense<double>(3, 3 * 6 * this.System.NumberOfElements);

            /// Insert the COG-Vector in all angular DOF proceeding and including the current Element
            for (int i = 2; i < ElementIndex; i += 6 * 3)
            {
                for (int j = 0; j < 3; j++)
                {
                    LocalVectorMatrix.InsertAtIndex(COGVector, i + j);
                }
            }
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
            Matrix<double> LocalMatrix = GetLocalMatrix(LocalStateVector);
            return ParentMatrix * LocalMatrix;
        }

        /// <summary>
        /// Calculates the "Local Matrix" (matrix with the partial diff rotation matrix and a unity matrix on the local states and the element rotation matrix on the others)
        /// </summary>
        /// <param name="LocalStateVector">Vector of the local states</param>
        /// <returns>Matrix with Rotational matrices on its diagonal</returns>
        private Matrix<double> GetLocalMatrix(Vector<double> LocalStateVector)
        {
            int NumElements = System.NumberOfElements;
            int ElementIndex = GetElementIndex() * 3;
            Matrix<double> LocalRotationalMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
            Matrix<double> LocalPartialDerivativeAlpha = Rotation.GetAlphaPartialDerivate(LocalStateVector[3]);
            Matrix<double> LocalPartialDerivativeBeta = Rotation.GetBetaPartialDerivate(LocalStateVector[4]);
            Matrix<double> LocalPartialDerivativeGamma = Rotation.GetGammaPartialDerivate(LocalStateVector[5]);
            Matrix<double> LocalMatrix = CreateMatrix.Dense<double>(NumElements * 6 * 3, NumElements * 6 * 3);
            for (int i = 0; i < LocalMatrix.ColumnCount; i += 3)
            {
                LocalMatrix.InsertAtIndex(LocalRotationalMatrix, i);
            }
            
            LocalMatrix.InsertAtIndex(CreateMatrix.DenseIdentity<double>(9), ElementIndex);
            LocalMatrix.InsertAtIndex(LocalPartialDerivativeAlpha, ElementIndex + 3 * 3);
            LocalMatrix.InsertAtIndex(LocalPartialDerivativeBeta, ElementIndex + 3 * 3 + 3);
            LocalMatrix.InsertAtIndex(LocalPartialDerivativeGamma, ElementIndex + 3 * 3 + 9);

            return LocalMatrix;
        }

        /// <summary>
        /// Calculates the derivative of the "Local Matrix", see 'GetLocalMatrix'
        /// </summary>
        /// <param name="LocalStateVector">Vector of the local states</param>
        /// <returns>Matrix with rotational matrices on its diagonal</returns>
        private Matrix<double> GetLocalMatrixDerivative(Vector<double> LocalStateVector)
        {
            int NumElements = System.NumberOfElements;
            int ElementIndex = GetElementIndex() * 3;
            Matrix<double> LocalRotationalMatrixDerivative = Rotation.GetTotalTimeDerivative(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5], LocalStateVector[3 + 6], LocalStateVector[4 + 6], LocalStateVector[5 + 6]);
            Matrix<double> LocalTotalPartialDerivativeAlpha = Rotation.GetAlphaTimeDerivativeOfPartial(LocalStateVector[3], LocalStateVector[3 + 6]);
            Matrix<double> LocalTotalPartialDerivativeBeta = Rotation.GetBetaTimeDerivativeOfPartial(LocalStateVector[4], LocalStateVector[4 + 6]);
            Matrix<double> LocalTotalPartialDerivativeGamma = Rotation.GetGammaTimeDerivativeOfPartial(LocalStateVector[5], LocalStateVector[5 + 6]);
            Matrix<double> LocalMatrixDerivative = CreateMatrix.Dense<double>(NumElements * 6 * 3, NumElements * 6 * 3);
            for (int i = 0; i < LocalMatrixDerivative.ColumnCount; i += 3)
            {
                LocalMatrixDerivative.InsertAtIndex(LocalRotationalMatrixDerivative, i);
            }
            LocalMatrixDerivative.InsertAtIndex(LocalTotalPartialDerivativeAlpha, ElementIndex + 3 * 3);
            LocalMatrixDerivative.InsertAtIndex(LocalTotalPartialDerivativeBeta, ElementIndex + 3 * 3 + 3);
            LocalMatrixDerivative.InsertAtIndex(LocalTotalPartialDerivativeGamma, ElementIndex + 3 * 3 + 9);

            return LocalMatrixDerivative;
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
        /// Get the first index of the element in the global state vector
        /// </summary>
        /// <returns></returns>
        private int GetElementIndex()
        {
            return 6 * (this.ElementId - 1);
        }
    }
}