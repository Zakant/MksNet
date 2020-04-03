using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Storage;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Mbs
{
    public class StateVector
    {
        protected double[] globalStateVector;

        protected List<StateVectorStorage> storages = new List<StateVectorStorage>();

        /// <summary>
        /// Creates a new statevector instance. <paramref name="globalStateVector"/> is passed by reference.
        /// </summary>
        /// <param name="globalStateVector">Array of global states. Passed by reference, they update if the underlaying array values are changed.</param>
        /// <param name="mappings">Mappings for each elements local full state vector to global compact state vector.</param>
        public StateVector(double[] globalStateVector, IEnumerable<Dictionary<int, int>> mappings)
        {
            this.globalStateVector = globalStateVector;
            foreach (var mapping in mappings)
                storages.Add(new StateVectorStorage(this.globalStateVector, mapping));
        }

        /// <summary>
        /// Retrieves the full six element state vector for the element with id <paramref name="elementId"/>.
        /// No index checking is performed for performance reasons.
        /// </summary>
        /// <param name="elementId">Id of the element.</param>
        /// <returns>Full six element state vector.</returns>
        public Vector<double> GetStateVectorForId(int elementId) => CreateVector.WithStorage<double>(storages[elementId]);
    }
}
