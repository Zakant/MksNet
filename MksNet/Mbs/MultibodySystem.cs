using MathNet.Numerics.LinearAlgebra;
using MksNet.Mbs.Elements;
using MksNet.Mbs.Parser;
using System;
using System.Collections.Generic;
using System.Text;
using System.Linq;

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
    }
}
