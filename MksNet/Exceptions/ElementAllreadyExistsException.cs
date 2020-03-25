using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Exceptions
{
    /// <summary>
    /// The exception that is thrown when the element type allready exists within the factory.
    /// </summary>
    [Serializable]
    public class ElementAllreadyExistsException : Exception
    {
        public ElementAllreadyExistsException() { }
        public ElementAllreadyExistsException(string message) : base(message) { }
        public ElementAllreadyExistsException(string message, Exception inner) : base(message, inner) { }
        protected ElementAllreadyExistsException(
          System.Runtime.Serialization.SerializationInfo info,
          System.Runtime.Serialization.StreamingContext context) : base(info, context) { }
    }
}
