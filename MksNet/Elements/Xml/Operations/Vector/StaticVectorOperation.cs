using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Elements.Xml.Operations.Vector
{
    internal class StaticVectorOperation : IVectorOperation
    {
        private Vector<double> vector;

        internal StaticVectorOperation(Vector<double> data)
        {
            vector = data;
        }

        public Vector<double> Resolve(ElementParameter parameter) => vector;
    }
}
