using MathNet.Numerics.LinearAlgebra.Storage;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs
{
    /// <summary>
    /// Stores a global compact state vector with mapping to local full size state vector.
    /// </summary>
    public class StateVectorStorage : VectorStorage<double>
    {

        public override bool IsDense => true;

        protected double[] globalStateVector;

        protected Dictionary<int, int> mapping;

        /// <summary>
        /// Creates a new state vector storage instace with a global compact state vector given by <paramref name="localStateVector"/>
        /// and a mapping to local full size state vector given by <paramref name="mapping"/>.
        /// </summary>
        /// <param name="globalStateVector">Global compact state vector.</param>
        /// <param name="mapping">Mapping from local full state vector to global compact state vector.</param>
        public StateVectorStorage(double[] globalStateVector, Dictionary<int, int> mapping) : base(6)
        {
            this.globalStateVector = globalStateVector;
            this.mapping = mapping;
        }

        public override double At(int index)
        {
            if (mapping.TryGetValue(index, out int key))
                return globalStateVector[key];
            else
                return 0;
        }
        public override void At(int index, double value)
        {
            if (mapping.TryGetValue(index, out int key))
                globalStateVector[key] = value;
        }
    }
}
