using System;
using System.Collections.Generic;
using System.Text;

namespace MksNet.Exceptions
{

    /// <summary>
    /// The exception that is thrown when a malformed or bad definition is encountered.
    /// </summary>
    [Serializable]
    public class BadDefinitionException : Exception
    {
        public BadDefinitionException() { }
        public BadDefinitionException(string message) : base(message) { }
        public BadDefinitionException(string message, Exception inner) : base(message, inner) { }
        protected BadDefinitionException(
          System.Runtime.Serialization.SerializationInfo info,
          System.Runtime.Serialization.StreamingContext context) : base(info, context) { }
    }
}
