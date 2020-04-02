using MathNet.Numerics.LinearAlgebra;
using MksNet.Mbs.Parser;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs
{
    public class MultibodySystem
    {
        /// <summary>
        /// Number of elements in the system.
        /// </summary>
        public int NumberOfElements { get; private set; }

        /// <summary>
        /// The magnitude and direction of gravitation.
        /// </summary>
        public Vector<double> GravitationVector { get; private set; }


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
