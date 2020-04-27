using MathNet.Numerics.LinearAlgebra;
using MksNet.Mbs.Elements;
using MksNet.Mbs.Parser;
using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;
using MksNet.Spartial;

namespace MksNet.Mbs
{
    public class MultibodySystem
    {

        private Vector<double>[] elementStateExistancesVectors;

        /// <summary>
        /// Elements of the multibody system.
        /// </summary>
        public IReadOnlyCollection<Element> Elements { get; internal set; }

        /// <summary>
        /// The magnitude and direction of gravitation.
        /// </summary>
        public Vector<double> GravitationVector { get; internal set; }

        /// <summary>
        /// Base frame of the multibody system.
        /// </summary>
        public Frame BaseFrame { get; internal set; }

        /// <summary>
        /// Count of all degrees of present.
        /// </summary>
        public int TotalDegreesOfFreedom { get; private set; } = 0;

        /// <summary>
        /// Indicates if a degree of freedom exists in the compact version.
        /// </summary>
        internal Vector<double> StateExistanceVector { get; private set; }

        private Matrix<double> KeepMatrix;

        /// <summary>
        /// Internal constructor for a multibody system. Use <see cref="MultibodySystem.Load(string)"/> or
        /// <see cref="MultibodySystem.LoadFromFile(string)"/> as a public creation interface.
        /// </summary>
        internal MultibodySystem() { }

        /// <summary>
        /// Creates a enumeration of mappings from local full state vector to global compact full state.
        /// </summary>
        /// <param name="includeTimeDerivatives"></param>
        /// <returns>Enumeration of mappings.</returns>
        public IEnumerable<Dictionary<int, int>> GenerateMappings(bool includeTimeDerivatives = true)
        {
            int offset = TotalDegreesOfFreedom;
            int globalIndex = 0;
            foreach (var element in Elements)
            {
                var mapping = new Dictionary<int, int>();
                foreach (var dof in element.BaseJoint.DegreesOfFreedom)
                {
                    mapping.Add((int)dof, globalIndex);
                    if (includeTimeDerivatives)
                        mapping.Add((int)dof + 6, globalIndex + offset);
                    globalIndex++;
                }
                yield return mapping;
            }
        }

        /// <summary>
        /// Load a mutlibody system defined in the file given by <paramref name="filename"/>.
        /// </summary>
        /// <param name="filename">Path of the multibody definition file.</param>
        /// <returns>Multibody system instance.</returns>
        public static MultibodySystem LoadFromFile(string filename) => Load(System.IO.File.ReadAllText(filename));

        /// <summary>
        /// Load a multibody system definied by the string given as <paramref name="definition"/>.
        /// </summary>
        /// <param name="definition">Definition string for a multibody system.</param>
        /// <returns>Multibody system instance.</returns>
        public static MultibodySystem Load(string definition) => MultibodySystemParser.Parse(definition);

        /// <summary>
        /// Initilizes the multibody system. After calling this method, no changes to elements should occure.
        /// </summary>
        internal void InitilizeSystem()
        {
            this.TotalDegreesOfFreedom = 0;
            // Setting up all element ids and system reference as well as the state existance vector.
            var data = new List<double>();
            elementStateExistancesVectors = new Vector<double>[Elements.Count];
            foreach ((int id, Element element) in Elements.Select((x, i) => (i, x)))
            {
                element.ElementId = id;
                element.System = this;
                TotalDegreesOfFreedom += element.BaseJoint.DegreesOfFreedom.Count;
                elementStateExistancesVectors[id] = CreateVector.Dense<double>(12, 0);
                foreach (var index in element.BaseJoint.DegreesOfFreedom.SelectMany(x => new int[] { (int)x, (int)x + 6 }))
                    elementStateExistancesVectors[id][index] = 1;
            }
            int offset = Elements.Count * 6;
            StateExistanceVector = CreateVector.Dense<double>(Elements.Count * 12, 0);
            Elements.SelectMany((element, index) =>
                                 element.BaseJoint.DegreesOfFreedom.SelectMany(y => new int[] { (index + 1) * (int)y, (index + 1) * (int)y + offset }))
            .ToList().ForEach(x => StateExistanceVector[x] = 1);
        }

        /// <summary>
        /// Get a state existance vector for the element with the id <paramref name="elementId"/>.
        /// </summary>
        /// <param name="elementId">Id of the element.</param>
        /// <returns>Vector indicating if a state exists in the global compact state vector.</returns>
        internal Vector<double> GetElementStateExistanceVector(int elementId) => elementStateExistancesVectors[elementId];

        public void GetGlobalKeepMatrix()
        {
            int NumberOfActiveDOF = 0;
            int NumberOfAvailableDOF = 0;
            int RowIndex;
            foreach(Element i in Elements)
            {
                NumberOfActiveDOF += Convert.ToInt32(GetElementStateExistanceVector(i.ElementId).Sum())/2;
                NumberOfAvailableDOF += 6;
            }
            Matrix<double> GlobalKeepMatrix = CreateMatrix.Dense<double>(NumberOfActiveDOF * 3, NumberOfAvailableDOF * 3);
            foreach(Element i in Elements)
            {
                Vector<double> DOFExist = GetElementStateExistanceVector(i.ElementId);
                RowIndex = 0;
                for (int index = 0; index < 6; index++)
                {
                    if (DOFExist[index] == 1)
                    {
                        GlobalKeepMatrix.InsertAtIndex(CreateMatrix.DenseIdentity<double>(3), index * 3, RowIndex);
                        RowIndex += 3;
                    }
                }
            }
            this.KeepMatrix = GlobalKeepMatrix;
        }

        /// <summary>
        /// Calculates the global mass matrix
        /// </summary>
        /// <param name="MassMatrix">The Mass matrix where the local matrices get inserted</param>
        /// <returns>The untransformed global mass matrix</returns>
        public Matrix<double> GetGlobalMassMatrix(Matrix<double> MassMatrix)
        {
            foreach (Element i in Elements)
            {
                MassMatrix = i.GetGlobalMassMatrix(MassMatrix);
            }
            return MassMatrix;
        }

        /// <summary>
        /// Calculates the global jacobian
        /// </summary>
        /// <param name="GlobalJacobian">The matrix where the element-matrices get inserted</param>
        /// <returns></returns>
        public Matrix<double> GetGlobalJacobian(Matrix<double> GlobalJacobian)
        {
            foreach (Element i in Elements)
            {
                GlobalJacobian = i.GetElementJacobian(GlobalJacobian, i.ElementId);
            }
            return GlobalJacobian;
        }

        /// <summary>
        /// Calculates the derivative of the global Jacobian
        /// </summary>
        /// <param name="GlobalJacobianDerivative">The matrix where the element-matrices get inserted</param>
        /// <returns></returns>
        private Matrix<double> GetGlobalJacobianDerivative(Matrix<double> GlobalJacobianDerivative)
        {
            foreach (Element i in Elements)
            {
                GlobalJacobianDerivative = i.GetElementJacobianDerivative(GlobalJacobianDerivative, i.ElementId);
            }
            return GlobalJacobianDerivative;
        }

        /// <summary>
        /// Calculates the vector of the free forces/moments
        /// </summary>
        /// <param name="FreeForceVector">The vector where the element-matrices get inserted</param>
        /// <returns></returns>
        public Vector<double> GetGlobalFreeForceVector(Vector<double> FreeForceVector)
        {
            foreach (Element i in Elements)
            {
                FreeForceVector = i.GetGlobalForceMomentVector(FreeForceVector);
            }
            return FreeForceVector;
        }

        /// <summary>
        /// Calculates the coriolis-vector
        /// </summary>
        /// <param name="CoriolisVector">The vector where the element-matrices get inserted</param>
        /// <param name="GlobalStateVector">The global state vector</param>
        /// <returns></returns>
        public Vector<double> GetCoriolisVector(Vector<double> CoriolisVector, StateVector GlobalStateVector)
        {
            Matrix<double> JacobianDerivative = GetGlobalJacobianDerivative(CreateMatrix.Dense<double>(Elements.Count * 6, Elements.Count * 6));
            Vector<double> Velocities = CreateVector.Dense<double>(3);/// GlobalStateVector.SubVector(GlobalStateVector.Count / 2, GlobalStateVector.Count / 2);
            CoriolisVector = JacobianDerivative * Velocities;
            Vector<double> LocalStateVector, AngularVelocity;
            int count = 3;
            foreach (Element i in Elements)
            {
                LocalStateVector = GlobalStateVector.GetStateVectorForId(i.ElementId);
                AngularVelocity = i.GetLocalAngularVelocity(LocalStateVector);
                Matrix<double> Roessel = RoesselMatrix.GetRoesselMatrix(AngularVelocity);
                CoriolisVector.AddAtIndex(Roessel * i.Inertia * AngularVelocity, count);
                count += 3;
            }
            return CoriolisVector;
        }

        /// <summary>
        /// Initilizes all needed vectors and matrices has to be run before simulation
        /// </summary>
        public void SetupElements()
        {
            foreach (Element i in this.Elements)
            {
                Vector<double> ElementStateExistanceVector = GetElementStateExistanceVector(i.ElementId);
                i.CreateKeepMatric(ElementStateExistanceVector);
                i.SetLocalPVectorCOG(this.KeepMatrix);
                i.SetLocalPVectorRotation(this.KeepMatrix);
            }
        }

        /// <summary>
        /// Updates all vectors and matrices of each element. Has to be run at every time step
        /// </summary>
        /// <param name="GlobalStateVector"></param>
        public void UpdateElements(StateVector GlobalStateVector)
        {
            foreach (Element i in this.Elements)
            {
                ///Vector<double> ls = GlobalStateVector.GetStateVectorForId(i.ElementId);
                Vector<double> ls = CreateVector.Dense<double>(6 * 2);
                i.Update(ls, this.KeepMatrix);
            }
        }
    }
}
