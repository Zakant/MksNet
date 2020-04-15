using System;
using System.Collections.Generic;
using System.Text;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics;
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
        /// The matrix that transforms matrices with scalar entries between all states and active states
        /// </summary>
        public Matrix<double> KeepMatrixScalar { get; internal set; }

        /// <summary>
        /// The matrix that transforms matrices with matrix entries between all states and active states
        /// </summary>
        public Matrix<double> KeepMatrixIdentity { get; internal set; }

        /// <summary>
        /// The local P matrix (Local rotation along the Diagonal with partial differentials on the rotational states)
        /// </summary>
        public Matrix<double> LocalPMatrixTranslation { get; internal set; }

        /// <summary>
        /// 
        /// </summary>
        public Matrix<double> LocalPMatrixRotation { get; internal set; }

        /// <summary>
        /// The time derivative local P matrix (Local rotation derivative along the Diagonal with total partial differentials on the rotational states)
        /// </summary>
        public Matrix<double> LocalPMatrixDerivativeTranslation { get; internal set; }

        /// <summary>
        /// 
        /// </summary>
        public Vector<double> LocalPVectorState { get; internal set; }

        /// <summary>
        /// 
        /// </summary>
        public Vector<double> LocalPVectorStateDerivative { get; internal set; }

        /// <summary>
        /// 
        /// </summary>
        public Vector<double> LocalPVectorCOG { get; internal set; }

        /// <summary>
        /// 
        /// </summary>
        public Vector<double> LocalPVectorRotation { get; internal set; }

        /// <summary>
        /// 
        /// </summary>
        public Matrix<double> LocalPMatrixRotationDerivative { get; internal set; }

        /// <summary>
        /// Rotation matrix of the element
        /// </summary>
        public Matrix<double> LocalRotationMatrix { get; internal set; }


        public Matrix<double> LocalRotationMatrixGamma;

        public Matrix<double> LocalRotationMatrixBeta;

        public Matrix<double> LocalRotationMatrixAlpha;

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
        public Matrix<double> LocalRotationMatrixPartialDiffTotalAlpha { get; internal set; }

        /// <summary>
        /// The total time derivative of the partial differential of the local rotation matrix with respect to the rotation along the y-axis
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartialDiffTotalBeta { get; internal set; }

        /// <summary>
        /// The total time derivative of the partial differential of the local rotation matrix with respect to the rotation along the x-axis
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartialDiffTotalGamma { get; internal set; }

        /// <summary>
        /// 
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartiallDiffAlphaTotal { get; internal set; }

        /// <summary>
        /// 
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartiallDiffBetaTotal { get; internal set; }

        /// <summary>
        /// The total derivative of the partial derivativ of the local rotation matrix
        /// </summary>
        public Matrix<double> LocalRotationMatrixPartiallDiffGammaTotal { get; internal set; }

        /// <summary>
        /// The total time derivative of the local rotation matrix
        /// </summary>
        public Matrix<double> LocalRotationMatrixTotalDerivative { get; internal set; }


        /// <summary>
        /// Create the transformation matrices for keeping the necessary local calculations. One for scalars, one for matrices
        /// </summary>
        /// <param name="ActiveStates">Vector indicating the active states</param>
        public void CreateKeepMatric(Vector<double> ActiveStates)
        {
            int ActiveStatesSum = Convert.ToInt32(ActiveStates.Sum());
            int NumAvailableDOF = ActiveStates.Count/2;
            int NumActiveDOF = ActiveStatesSum / 2;

            ///this.KeepMatrixScalar = CreateMatrix.Dense<double>(NumAvailableDOF, NumActiveDOF);
            this.KeepMatrixScalar = CreateMatrix.Dense<double>(NumAvailableDOF, NumAvailableDOF);
            ///int localCounter = 0;
            ///for (int i = 0; i < NumAvailableDOF; i++)
            ///{
            ///    if (ActiveStates[i] == 1 || true)
            ///    {
            ///Vector<double> identity = CreateVector.Dense<double>(NumAvailableDOF);
            ///identity[i] = 1.0;
            ///this.KeepMatrixScalar.InsertAtIndex(identity, localCounter);
            ///localCounter++;
            ///}
            ///}
            ///this.KeepMatrixIdentity = CreateMatrix.Dense<double>(NumActiveDOF * 3, NumAvailableDOF * 3);
            this.KeepMatrixIdentity = CreateMatrix.DenseIdentity<double>(NumAvailableDOF * 3, NumAvailableDOF * 3);
            ///localCounter = 0;
            ///for (int i = 0; i < NumAvailableDOF; i++)
            ///{
            ///if (ActiveStates[i] == 1 || true)
            ///{
            ///this.KeepMatrixIdentity.InsertAtIndex(CreateMatrix.DenseIdentity<double>(3), i * 3, localCounter);
            ///localCounter += 3;
            ///}
            ///}
        }

        /// <summary>
        /// 
        /// </summary>
        public void SetLocalPVectorCOG()
        {
            int ElementIndex = GetElementIndex();
            Vector<double> TestCOG = CreateVector.DenseOfArray<double>(new double[3] { 1, 2, 3 });
            ///this.LocalPVectorCOG = this.KeepMatrixIdentity.Transpose() * GetLocalVectorMatrix(ElementIndex, Cog.Offset); /// Offset from parent origin to local origin missing
            this.LocalPVectorCOG = GetLocalVectorMatrix(ElementIndex, TestCOG);
            this.LocalPVectorCOG.InsertAtIndex(IdentityVectors.X, ElementIndex);
            this.LocalPVectorCOG.InsertAtIndex(IdentityVectors.Y, ElementIndex + 3);
            this.LocalPVectorCOG.InsertAtIndex(IdentityVectors.Z, ElementIndex + 6);
            this.LocalPVectorCOG = this.KeepMatrixIdentity * this.LocalPVectorCOG;
        }

        /// <summary>
        /// 
        /// </summary>
        public void SetLocalPVectorRotation()
        {
            int ElementIndex = GetElementIndex();
            ///this.LocalPVectorRotation = this.KeepMatrixIdentity * GetLocalVectorMatrixIdentity(ElementIndex);
            this.LocalPVectorRotation = GetLocalVectorMatrixIdentity(ElementIndex);
        }

        /// <summary>
        /// Update the element with the state vector given in <paramref name="LocalStateVector"/>.
        /// </summary>
        /// <param name="LocalStateVector">Statevector used for update.</param>
        public void Update(Vector<double> LocalStateVector)
        {
            UpdateLocalRotationMatrices(LocalStateVector);
            UpdatePMatrix();
            UpdatePMatrixDerivative();
            UpdatePVectorState(LocalStateVector);
            UpdatePVectorStateDerivative(LocalStateVector);
        }

        /// <summary>
        /// Updates the local P-Matrix
        /// </summary>
        private void UpdatePMatrix()
        {
            this.LocalPMatrixTranslation = this.KeepMatrixIdentity * GetLocalMatrixTranslation() * this.KeepMatrixIdentity.Transpose();
            this.LocalPMatrixRotation = this.KeepMatrixIdentity * GetLocalMatrixRotation() * this.KeepMatrixIdentity.Transpose();
            this.LocalPMatrixDerivativeTranslation = this.KeepMatrixIdentity * GetLocalMatrixRotationDerivative() * this.KeepMatrixIdentity.Transpose();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="LocalStateVector"></param>
        private void UpdatePMatrixDerivative()
        {
            this.LocalPMatrixDerivativeTranslation = this.KeepMatrixIdentity * GetLocalMatrixDerivative() * this.KeepMatrixIdentity.Transpose();
        }

        /// <summary>
        /// Updats the local P-Vector (without transformation
        /// </summary>
        /// <param name="LocalStateVector">Vector with local states</param>
        private void UpdatePVectorState(Vector<double> LocalStateVector)
        {
            this.LocalPVectorState = this.KeepMatrixIdentity * GetLocalVectorMatrix(GetElementIndex(), LocalStateVector.SubVector(0, 3)); /// Vector from Parent origin to Local origin is missing
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="LocalStateVector"></param>
        private void UpdatePVectorStateDerivative(Vector<double> LocalStateVector)
        {
            this.LocalPVectorStateDerivative = this.KeepMatrixIdentity * GetLocalVectorMatrix(GetElementIndex(), LocalStateVector.SubVector(6, 3)); /// Vector from Parent origin to Local origin is missing
        }

        /// <summary>
        /// Updates all needed rotational matrices of the element
        /// </summary>
        /// <param name="LocalStateVector">Vector with local states</param>
        private void UpdateLocalRotationMatrices(Vector<double> LocalStateVector)
        {
            this.LocalRotationMatrix = Rotation.GetXYZ(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5]);
            this.LocalRotationMatrixAlpha = Rotation.GetX(LocalStateVector[3]);
            this.LocalRotationMatrixBeta = Rotation.GetY(LocalStateVector[4]);
            this.LocalRotationMatrixGamma = Rotation.GetZ(LocalStateVector[5]);
            this.LocalRotationMatrixPartialDiffAlpha = Rotation.GetAlphaPartialDerivate(LocalStateVector[3]);
            this.LocalRotationMatrixPartialDiffBeta = Rotation.GetBetaPartialDerivate(LocalStateVector[4]);
            this.LocalRotationMatrixPartialDiffGamma = Rotation.GetGammaPartialDerivate(LocalStateVector[5]);
            this.LocalRotationMatrixPartialDiffTotalAlpha = Rotation.GetAlphaTimeDerivativeOfPartial(LocalStateVector[3], LocalStateVector[9]);
            this.LocalRotationMatrixPartialDiffTotalBeta = Rotation.GetBetaTimeDerivativeOfPartial(LocalStateVector[3], LocalStateVector[10]);
            this.LocalRotationMatrixPartialDiffTotalGamma = Rotation.GetGammaTimeDerivativeOfPartial(LocalStateVector[5], LocalStateVector[11]);
            this.LocalRotationMatrixPartiallDiffAlphaTotal = Rotation.GetTotalTimeDerivativeToAlpha(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5], LocalStateVector[9], LocalStateVector[10], LocalStateVector[11]);
            this.LocalRotationMatrixPartiallDiffBetaTotal = Rotation.GetTotalTimeDerivativeToBeta(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5], LocalStateVector[9], LocalStateVector[10], LocalStateVector[11]);
            this.LocalRotationMatrixPartiallDiffGammaTotal = Rotation.GetTotalTimeDerivativeToGamma(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5], LocalStateVector[9], LocalStateVector[10], LocalStateVector[11]);
            this.LocalRotationMatrixTotalDerivative = Rotation.GetTotalTimeDerivative(LocalStateVector[3], LocalStateVector[4], LocalStateVector[5], LocalStateVector[9], LocalStateVector[10], LocalStateVector[11]);
        }

        /// <summary>
        /// Calculates the product of all previous parent matrices
        /// </summary>
        /// <returns>The product of all parent matrices</returns>
        public Matrix<double> GetParentMatrixTranslation()
        {
            if (Parent == null)
            {
                return this.LocalPMatrixTranslation;
            }
            else
            {
                return Parent.GetParentMatrixTranslation() * this.LocalPMatrixTranslation;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public Matrix<double> GetParentMatrixTranslationDerivarive()
        {
            if (Parent == null)
            {
                return this.LocalPMatrixDerivativeTranslation;
            }
            else
            {
                return Parent.GetParentMatrixTranslationDerivarive() * this.LocalPMatrixTranslation + Parent.GetParentMatrixTranslation() * this.LocalPMatrixDerivativeTranslation;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public Matrix<double> GetParentMatrixRotation()
        {
            if (Parent == null)
            {
                return this.LocalPMatrixRotation;
            }
            else
            {
                return Parent.GetParentMatrixRotation() * this.LocalPMatrixRotation;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public Matrix<double> GetParentMatrixRotationDerivative()
        {
            if (Parent == null)
            {
                return Parent.GetParentMatrixRotation() * this.LocalPMatrixRotationDerivative;
            }
            else
            {
                return Parent.GetParentMatrixRotationDerivative() * this.LocalPMatrixRotation + Parent.GetParentMatrixRotation() * this.LocalPMatrixRotationDerivative;
            }
        }

        /// <summary>
        /// Gets the parent vector
        /// </summary>
        /// <returns>The parent vector</returns>
        public Vector<double> GetParentVector()
        {
            if (Parent == null)
            {
                return GetParentMatrixTranslation() * this.LocalPVectorState;
            }
            else
            {
                return Parent.GetParentVector() + GetParentMatrixTranslation() * this.LocalPVectorState;
            }
        }

        /// <summary>
        /// Get the time derivative of the parent vector
        /// </summary>
        /// <returns></returns>
        public Vector<double> GetParentVectorDerivative()
        {
            if (Parent == null)
            {
                return GetParentMatrixTranslationDerivarive() * LocalPVectorState + GetParentMatrixTranslation() * LocalPVectorStateDerivative;
            }
            else
            {
                return Parent.GetParentVectorDerivative() + GetParentMatrixTranslationDerivarive() * LocalPVectorState + GetParentMatrixTranslation() * LocalPVectorStateDerivative;
            }
        }

        /// <summary>
        /// Calculates the product of all parent rotation matrices
        /// </summary>
        /// <returns>The matrix product</returns>
        public Matrix<double> GetParentMatrixProduct()
        {
            return Parent.GetParentMatrixProductWLocal();
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public Matrix<double> GetParentMatrixProductWLocal()
        {
            return Parent.GetParentMatrixProduct() * this.LocalRotationMatrix;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public Matrix<double> GetParentMatrixDerivativeProduct()
        {
            return Parent.GetParentMatrixDerivativeProduct()* this.LocalPMatrixTranslation + GetParentMatrixTranslation() * this.LocalPMatrixDerivativeTranslation;
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
        public Vector<double> GetGlobalForceMomentVector(Vector<double> GlobalForceVector)
        {
            int ElementIndex = GetElementIndex();
            int ParentIndex = Parent.GetElementIndex();
            Vector<double> LocalVector = GetLocalForceMomentVector();
            GlobalForceVector.AddAtIndex(LocalVector, ElementIndex);
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
            return GetParentMatrixProduct() * LocalAngularVelocities;
        }

        /// <summary>
        /// Calculates and inserts the local jacobians of the element and its children into the global jacobian
        /// </summary>
        /// <param name="GlobalJacobian">The global jacobian</param>
        /// <param name="GlobalIndex">Index in the GlobalJacobian</param>
        /// <returns></returns>
        public Matrix<double> GetElementJacobian(Matrix<double> GlobalJacobian, int GlobalIndex)
        {
            Vector<double> LocalTranslationalJacobian = GetTranslationalJacobianMatrix();
            Vector<double> LocalRotationalJacobian = GetRotationalJacobian();

            for (int i = 0; i * 3 < LocalTranslationalJacobian.Count; i++)
            {
                GlobalJacobian.InsertAtIndex(LocalTranslationalJacobian.SubVector(i * 3, 3), i, GlobalIndex * 3);
                GlobalJacobian.InsertAtIndex(LocalRotationalJacobian.SubVector(i * 3, 3), i, GlobalIndex * 3 + 3);
            }

            ///GlobalJacobian.InsertAtIndex(LocalTranslationalJacobian, 0, GlobalIndex);
            ///GlobalJacobian.InsertAtIndex(LocalRotationalJacobian, 0, GlobalIndex + 3);

            return GlobalJacobian;
        }

        /// <summary>
        /// Calculates the time derivative of the local jacobian matrix
        /// </summary>
        /// <param name="GlobalJacobianDerivative">Time derivative of the global jacobian</param>
        /// <returns></returns>
        public Matrix<double> GetElementJacobianDerivative(Matrix<double> GlobalJacobianDerivative, int GlobalIndex)
        {
            Vector<double> LocalTranslationalJacobianDerivative = GetTranslationalJacobianDerivative();

            Vector<double> LocalRotationalJacobianDerivative = GetRotationalJacobianDerivative();

            for (int i = 0; i * 3 < LocalTranslationalJacobianDerivative.Count; i++)
            {
                GlobalJacobianDerivative.InsertAtIndex(LocalTranslationalJacobianDerivative.SubVector(i * 3, 3), i, GlobalIndex * 3);
                GlobalJacobianDerivative.InsertAtIndex(LocalRotationalJacobianDerivative.SubVector(i * 3, 3), i, GlobalIndex * 3 + 3);
            }

            ///GlobalJacobianDerivative.InsertAtIndex(LocalTranslationalJacobianDerivative, 0, GlobalIndex);
            ///GlobalJacobianDerivative.InsertAtIndex(LocalRotationalJacobianDerivative, 0, GlobalIndex + 3);

            return GlobalJacobianDerivative;

        }

        /// <summary>
        /// Computates the Translational Jacobian of the Element
        /// </summary>
        /// <returns></returns>
        public Vector<double> GetTranslationalJacobianMatrix()
        {
            return GetParentVector() + GetParentMatrixTranslation() * this.LocalPVectorCOG;
        }

        /// <summary>
        /// Calculates the rotational jacobian of the element
        /// </summary>
        /// <param name="stateVector">Global state vector</param>
        /// <param name="ParentRotationalJacobian">The jacobian of the previous element</param>
        /// <param name="ParentRotatinalMatrixProduct">The product of all rotational matrices of the previous elements</param>
        /// <returns></returns>
        public Vector<double> GetRotationalJacobian()
        {
            return GetParentMatrixRotation() * this.LocalPVectorRotation;
        }

        /// <summary>
        /// Calculates the derivative of the local translational jacobian
        /// </summary>
        /// <returns></returns>
        public Vector<double> GetTranslationalJacobianDerivative()
        {
            return GetParentVectorDerivative() + GetParentMatrixTranslationDerivarive() * this.LocalPVectorState + GetParentMatrixTranslation() * this.LocalPVectorStateDerivative;
        }

        /// <summary>
        /// Calculates the derivative of the local rotational jacobian
        /// </summary>
        /// <returns></returns>
        public Vector<double> GetRotationalJacobianDerivative()
        {
            if (Parent == null)
            {
                return this.LocalPVectorRotation;
            }
            else
            {
                return GetParentMatrixRotationDerivative() * this.LocalPVectorRotation;
            }
        }

        /// <summary>
        /// Creates the local vector for the translational jacobian with the local Vector
        /// </summary>
        /// <param name="ElementIndex">Index of the element in the global vector</param>
        /// <param name="Vector">Local vector</param>
        /// <returns>Matrix containing the COG-vectors and unity vectors</returns>
        private Vector<double> GetLocalVectorMatrix(int ElementIndex, Vector<double> Vector)
        {
            Vector<double> LocalVectorMatrix = CreateVector.Dense<double>(3 * 6 * this.System.Elements.Count);
            for (int i = 9; i < ElementIndex + 6 * 3; i += 6 * 3)
            {
                for (int j = 0; j < 9; j += 3)
                {
                    LocalVectorMatrix.InsertAtIndex(Vector, i + j);
                }
            }
            return LocalVectorMatrix;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="ElementIndex"></param>
        /// <returns></returns>
        private Vector<double> GetLocalVectorMatrixIdentity(int ElementIndex)
        {
            Vector<double> LocalVectorMatrix = CreateVector.Dense<double>(3 * 6 * this.System.Elements.Count);

            /// Insert the COG-Vector in all angular DOF proceeding and including the current Element
            for (int i = 3 * 3; i < ElementIndex + 6 * 3; i += 6 * 3)
            {
                LocalVectorMatrix.InsertAtIndex(IdentityVectors.X, i);
                LocalVectorMatrix.InsertAtIndex(IdentityVectors.Y, i + 3);
                LocalVectorMatrix.InsertAtIndex(IdentityVectors.Z, i + 6);
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
            Matrix<double> LocalMatrix = GetLocalMatrixTranslation();
            return ParentMatrix * LocalMatrix;
        }

        /// <summary>
        /// Calculates the "Local Matrix" (matrix with the partial diff rotation matrix and a unity matrix on the local states and the element rotation matrix on the others)
        /// </summary>
        /// <param name="LocalStateVector">Vector of the local states</param>
        /// <returns>Matrix with Rotational matrices on its diagonal</returns>
        private Matrix<double> GetLocalMatrixTranslation()
        {
            int NumElements = System.Elements.Count;
            int ElementIndex = GetElementIndex() * 3;
            Matrix<double> LocalMatrixTranslation = CreateMatrix.Dense<double>(NumElements * 6 * 3, NumElements * 6 * 3);
            for (int i = 0; i < LocalMatrixTranslation.ColumnCount; i += 3)
            {
                LocalMatrixTranslation.InsertAtIndex(LocalRotationMatrix * 2, i);
            }

            LocalMatrixTranslation.InsertAtIndex(CreateMatrix.DenseIdentity<double>(9), ElementIndex);
            LocalMatrixTranslation.InsertAtIndex(LocalRotationMatrixPartialDiffAlpha, ElementIndex + 3 * 3);
            LocalMatrixTranslation.InsertAtIndex(LocalRotationMatrixPartialDiffBeta * 3, ElementIndex + 3 * 3 + 3);
            LocalMatrixTranslation.InsertAtIndex(LocalRotationMatrixPartialDiffGamma * 4, ElementIndex + 3 * 3 + 6);

            return LocalMatrixTranslation;
        }

        /// <summary>
        /// Calculates a matrix with the local rotation Matrix on its diagonal and gamma * beta, beta and identity on the elements rotational degrees of freedom 
        /// </summary>
        /// <returns></returns>
        private Matrix<double> GetLocalMatrixRotation()
        {
            int NumElements = System.Elements.Count;
            int ElementIndex = GetElementIndex() * 3;
            Matrix<double> LocalMatrixRotation = CreateMatrix.DenseIdentity<double>(NumElements * 6 * 3, NumElements * 6 * 3);
            for (int i = ElementIndex + 3 * 6; i < LocalMatrixRotation.ColumnCount; i += 3 + 6 * 3)
            {
                LocalMatrixRotation.InsertAtIndex(LocalRotationMatrix, i);
            }

            LocalMatrixRotation.InsertAtIndex(CreateMatrix.DenseIdentity<double>(9), ElementIndex);
            LocalMatrixRotation.InsertAtIndex(this.LocalRotationMatrixGamma * this.LocalRotationMatrixBeta, ElementIndex + 3 * 3);
            LocalMatrixRotation.InsertAtIndex(this.LocalRotationMatrixBeta, ElementIndex + 3 * 3 + 3);
            LocalMatrixRotation.InsertAtIndex(CreateMatrix.DenseIdentity<double>(3), ElementIndex + 3 * 3 + 6);
            return LocalMatrixRotation;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        private Matrix<double> GetLocalMatrixRotationDerivative()
        {
            int NumElements = System.Elements.Count;
            int ElementIndex = GetElementIndex() * 3;
            Matrix<double> LocalMatrixRotation = CreateMatrix.DenseIdentity<double>(NumElements * 6 * 3, NumElements * 6 * 3);
            for (int i = ElementIndex; i < LocalMatrixRotation.ColumnCount; i += 3)
            {
                LocalMatrixRotation.InsertAtIndex(LocalRotationMatrixTotalDerivative, i);
            }
            LocalMatrixRotation.InsertAtIndex(LocalRotationMatrixPartialDiffTotalGamma * LocalRotationMatrixPartialDiffBeta +  LocalRotationMatrixPartialDiffGamma * LocalRotationMatrixPartialDiffTotalBeta, ElementIndex + 3 * 3);
            LocalMatrixRotation.InsertAtIndex(LocalRotationMatrixPartialDiffTotalBeta, ElementIndex + 3 * 3 + 3);
            LocalMatrixRotation.InsertAtIndex(CreateMatrix.Dense<double>(3, 3), ElementIndex + 3 * 3 + 6);
            return LocalMatrixRotation;
        }

        /// <summary>
        /// Calculates the derivative of the "Local Matrix", see 'GetLocalMatrix'
        /// </summary>
        /// <returns>Matrix with rotational matrices on its diagonal</returns>
        private Matrix<double> GetLocalMatrixDerivative()
        {
            int NumElements = System.Elements.Count;
            int ElementIndex = GetElementIndex() * 3;
            Matrix<double> LocalMatrixRotationDerivative = CreateMatrix.Dense<double>(NumElements * 6 * 3, NumElements * 6 * 3);
            for (int i = 0; i < LocalMatrixRotationDerivative.ColumnCount; i += 3)
            {
                LocalMatrixRotationDerivative.InsertAtIndex(this.LocalRotationMatrixTotalDerivative, i);
            }
            LocalMatrixRotationDerivative.InsertAtIndex(LocalRotationMatrixPartialDiffTotalGamma * LocalRotationMatrixPartialDiffBeta, ElementIndex + 3 * 3);
            LocalMatrixRotationDerivative.InsertAtIndex(LocalRotationMatrixPartialDiffBeta, ElementIndex + 3 * 3 + 3);
            LocalMatrixRotationDerivative.InsertAtIndex(CreateMatrix.DenseIdentity<double>(3), ElementIndex + 3 * 3 + 6);

            return LocalMatrixRotationDerivative;
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
            return 6 * (this.ElementId);
        }
    }
}