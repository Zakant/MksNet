using MathNet.Numerics.LinearAlgebra;
using MksNet.Mbs.Elements.Xml.Operations.Scalar;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace MksNet.Mbs.Elements.Xml.Operations.Vector
{
    internal class VectorOperation : IVectorOperation
    {
        private List<IScalarOperation> vector;

        internal VectorOperation(IEnumerable<IScalarOperation> scalars)
        {
            vector = scalars.ToList();
        }

        public Vector<double> Resolve(ElementParameter parameter) => CreateVector.DenseOfEnumerable(vector.Select(x => x.Resolve(parameter)));
    }
}
