using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Exceptions
{

    /// <summary>
    /// The exception that is thrown when the requested element type can not be found.
    /// </summary>
    [Serializable]
    public class ElementNotFoundException : Exception
    {
        public ElementNotFoundException() { }
        public ElementNotFoundException(string message) : base(message) { }
        public ElementNotFoundException(string message, Exception inner) : base(message, inner) { }
        protected ElementNotFoundException(
          System.Runtime.Serialization.SerializationInfo info,
          System.Runtime.Serialization.StreamingContext context) : base(info, context) { }
    }
}
