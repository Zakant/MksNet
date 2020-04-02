using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs
{
    /// <summary>
    /// Contains parameters used for <see cref="Element"/> or <see cref="MultibodySystem"/> creation.
    /// </summary>
    public class Parameter
    {
        /// <summary>
        /// Empty parameter instace.
        /// </summary>
        public static Parameter Empty { get; } = new Parameter();

        private Dictionary<string, double> scalarParameters = new Dictionary<string, double>();

        private Dictionary<string, Vector<double>> vectorParameters = new Dictionary<string, Vector<double>>();

        private Dictionary<string, Matrix<double>> matrixParameters = new Dictionary<string, Matrix<double>>();

        /// <summary>
        /// Add a new scalar parameter.
        /// </summary>
        /// <param name="name">Name of the parameter.</param>
        /// <param name="value">Scalar parameter value.</param>
        public void Add(string name, double value) => scalarParameters.Add(name, value);

        /// <summary>
        /// Add a new vector parameter.
        /// </summary>
        /// <param name="name">Name of the parameter.</param>
        /// <param name="value">Vector parameter value.</param>
        public void Add(string name, Vector<double> value) => vectorParameters.Add(name, value);

        /// <summary>
        /// Add a new matrix parameter.
        /// </summary>
        /// <param name="name">Name of the parameter.</param>
        /// <param name="value">Matrix parameter value.</param>
        public void Add(string name, Matrix<double> value) => matrixParameters.Add(name, value);

        /// <summary>
        /// Check if a scalar parameter with the given name is present.
        /// </summary>
        /// <param name="name">Name of the parameter.</param>
        /// <returns>True if the parameter is present, otherwise false.</returns>
        public bool HasScalar(string name) => scalarParameters.ContainsKey(name);

        /// <summary>
        /// Check if a vector parameter with the given name is present.
        /// </summary>
        /// <param name="name">Name of the parameter.</param>
        /// <returns>True if the parameter is present, otherwise false.</returns>
        public bool HasVector(string name) => vectorParameters.ContainsKey(name);

        /// <summary>
        /// Check if a matrix parameter with the given name is present.
        /// </summary>
        /// <param name="name">Name of the parameter.</param>
        /// <returns>True if the parameter is present, otherwise false.</returns>
        public bool HasMatrix(string name) => matrixParameters.ContainsKey(name);

        /// <summary>
        /// Get a scalar parameter with the given <paramref name="name"/>.
        /// </summary>
        /// <param name="name">Name of the parameter.</param>
        /// <returns>The scalar value.</returns>
        public double GetScalar(string name) => scalarParameters[name];

        /// <summary>
        /// Get a vector parameter with the given <paramref name="name"/>.
        /// </summary>
        /// <param name="name">Name of the parameter.</param>
        /// <returns>The vector value.</returns>
        public Vector<double> GetVector(string name) => vectorParameters[name];

        /// <summary>
        /// Get a matrix parameter with the given <paramref name="name"/>.
        /// </summary>
        /// <param name="name">Name of the parameter.</param>
        /// <returns>The matrix value.</returns>
        public Matrix<double> GetMatrix(string name) => matrixParameters[name];

    }
}
