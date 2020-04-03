using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Exceptions
{

    /// <summary>
    /// The exception that is thrown when a parameter is not found..
    /// </summary>
    [Serializable]
    public class ParameterNotFoundException : Exception
    {
        public ParameterNotFoundException() { }
        public ParameterNotFoundException(string message) : base(message) { }
        public ParameterNotFoundException(string message, Exception inner) : base(message, inner) { }
        protected ParameterNotFoundException(
          System.Runtime.Serialization.SerializationInfo info,
          System.Runtime.Serialization.StreamingContext context) : base(info, context) { }
    }
}
