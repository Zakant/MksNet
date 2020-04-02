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
        /// Internal constructor for a multibody system. Use <see cref="MultibodySystem.Load(string)"/> or
        /// <see cref="MultibodySystem.LoadFromFile(string)"/> as a public creation interface.
        /// </summary>
        internal MultibodySystem() { }

        /// <summary>
        /// Initilizes the multibody system. After calling this method, no changes to elements should occure.
        /// </summary>
        internal void InitilizeSystem()
        {
            this.TotalDegreesOfFreedom = 0;
            // Setting up all element ids and system reference.
            foreach ((int id, Element element) in Elements.Select((x, i) => (i, x)))
            {
                element.ElementId = id;
                element.System = this;
                TotalDegreesOfFreedom += element.BaseJoint.DegreesOfFreedom.Count;
            }
            // Construct state vector.
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
    }
}
