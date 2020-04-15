using MksNet.Mbs;
using MksNet.Mbs.Elements;
using MksNet.Mbs.Joints;
using System;
using MathNet.Numerics.LinearAlgebra;

namespace MksNetCli
{
    class Program
    {
        static void Main(string[] args)
        {
            Console.WriteLine("Hello World!");

            ElementFactory.Instance.LoadFolderDefinition(@".\Definitions\Elements");
            JointFactory.Instance.LoadFolderDefinition(@".\Definitions\Joints");
            MultibodySystem MBS = MultibodySystem.LoadFromFile(@".\Definitions\Systems\SimplePendulum.xml");
            double[] vec = new double[MBS.Elements.Count * 6 * 2];
            StateVector SV = new StateVector(vec, MBS.GenerateMappings());
            MBS.SetupElements();
            MBS.UpdateElements(SV);
            Matrix<double> GlobalJacobian = CreateMatrix.Dense<double>(6, 6);
            Matrix<double> GlobalJacobianDerivative = CreateMatrix.Dense<double>(6, 6);
            foreach(Element i in MBS.Elements)
            {
                GlobalJacobian = i.GetElementJacobian(GlobalJacobian, 0);
                GlobalJacobianDerivative = i.GetElementJacobianDerivative(GlobalJacobianDerivative, 0);
            }
        }
    }
}
