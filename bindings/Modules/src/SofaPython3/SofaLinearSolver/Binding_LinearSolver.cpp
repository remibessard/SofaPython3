/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2021 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <SofaPython3/SofaLinearSolver/Binding_LinearSolver.h>
#include <SofaPython3/SofaLinearSolver/Binding_LinearSolver_doc.h>
#include <pybind11/pybind11.h>

#include <SofaPython3/Sofa/Core/Binding_Base.h>
#include <SofaPython3/Sofa/Core/Binding_LinkPath.h>
#include <SofaPython3/Sofa/Core/Binding_BaseObject.h>
#include <SofaPython3/PythonFactory.h>

#include <sofa/component/linearsolver/iterative/MatrixLinearSolver.h>
#include <pybind11/eigen.h>

namespace py { using namespace pybind11; }

namespace sofapython3 {

using EigenSparseMatrix = Eigen::SparseMatrix<SReal, Eigen::RowMajor>;
using EigenMatrixMap = Eigen::Map<Eigen::SparseMatrix<SReal, Eigen::RowMajor> >;
using Vector = Eigen::Matrix<SReal,Eigen::Dynamic, 1>;
using EigenVectorMap = Eigen::Map<Vector>;

template<class TBlock>
EigenSparseMatrix toEigen(sofa::linearalgebra::CompressedRowSparseMatrix<TBlock>& matrix)
{
    sofa::linearalgebra::CompressedRowSparseMatrix<typename TBlock::Real> filtered;
    filtered.copyNonZeros(matrix);
    filtered.compress();
    return EigenMatrixMap(filtered.rows(), filtered.cols(), filtered.getColsValue().size(),
            (EigenMatrixMap::StorageIndex*)filtered.rowBegin.data(), (EigenMatrixMap::StorageIndex*)filtered.colsIndex.data(), filtered.colsValue.data());
}

template<>
EigenSparseMatrix toEigen<SReal>(sofa::linearalgebra::CompressedRowSparseMatrix<SReal>& matrix)
{
    return EigenMatrixMap(matrix.rows(), matrix.cols(), matrix.getColsValue().size(),
                    (EigenMatrixMap::StorageIndex*)matrix.rowBegin.data(), (EigenMatrixMap::StorageIndex*)matrix.colsIndex.data(), matrix.colsValue.data());
}


template<class TBlock>
void bindLinearSolvers(py::module &m)
{
    using CRS = sofa::linearalgebra::CompressedRowSparseMatrix<TBlock>;
    using CRSLinearSolver = sofa::component::linearsolver::MatrixLinearSolver<CRS, sofa::linearalgebra::FullVector<SReal> >;

    const std::string typeName = CRSLinearSolver::GetClass()->className + CRSLinearSolver::GetCustomTemplateName();

    py::class_<CRSLinearSolver,
               sofa::core::objectmodel::BaseObject,
               sofapython3::py_shared_ptr<CRSLinearSolver> > c(m, typeName.c_str(), sofapython3::doc::linearsolver::linearSolverClass);

    c.def("A", [](CRSLinearSolver& self) -> EigenSparseMatrix
    {
        if (CRS* matrix = self.getSystemMatrix())
        {
            return toEigen(*matrix);
        }
        return {};
    }, sofapython3::doc::linearsolver::linearSolver_A);

    c.def("b", [](CRSLinearSolver& self) -> Vector
    {
        if (auto* vector = self.getSystemRHVector())
        {
            return EigenVectorMap(vector->ptr(), vector->size());
        }
        return {};
    }, sofapython3::doc::linearsolver::linearSolver_b);

    c.def("x", [](CRSLinearSolver& self) -> Vector
    {
        if (auto* vector = self.getSystemLHVector())
        {
            return EigenVectorMap(vector->ptr(), vector->size());
        }
        return {};
    }, sofapython3::doc::linearsolver::linearSolver_x);


    /// register the binding in the downcasting subsystem
    PythonFactory::registerType<CRSLinearSolver>([](sofa::core::objectmodel::Base* object)
    {
        return py::cast(dynamic_cast<CRSLinearSolver*>(object));
    });
}

void bindMatrixFreeLinearSolvers(py::module& m)
{
    //< component::linearsolver::GraphScatkteredMatrix, component::linearsolver::GraphScatteredVector
    using GSV = sofa::component::linearsolver::GraphScatteredVector;
    using GSLinearSolver = sofa::component::linearsolver::MatrixLinearSolver<sofa::component::linearsolver::GraphScatteredMatrix, GSV >;

    const std::string typeName = GSLinearSolver::GetClass()->className + GSLinearSolver::GetCustomTemplateName();

    py::class_<GSLinearSolver,
        sofa::core::objectmodel::BaseObject,
        sofapython3::py_shared_ptr<GSLinearSolver> > c(m, typeName.c_str(), sofapython3::doc::linearsolver::linearSolverClass);

   c.def("x", [](GSLinearSolver& self, py::object& mstateLink) -> Vector
        {
            if (const GSV* vector = self.getSystemLHVector())
            {
                if (py::isinstance<py::str>(mstateLink))
                {
                    sofa::core::behavior::MechanicalState<sofa::defaulttype::Rigid3Types>* rmstate = nullptr;
                    self.getContext()->findLinkDest(rmstate, mstateLink.cast<std::string>(), nullptr);
                    if (rmstate)
                    {
                        auto bvec = sofa::helper::getWriteAccessor(*vector->id()[rmstate].write()).wref();
                        return EigenVectorMap(bvec.data()->ptr(), sofa::defaulttype::Rigid3Types::Deriv::size() * bvec.size());
                    }
                    sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3Types>* mstate = nullptr;
                    self.getContext()->findLinkDest(mstate, mstateLink.cast<std::string>(), nullptr);
                    if (mstate)
                    {
                        auto bvec = sofa::helper::getWriteAccessor(*vector->id()[mstate].write()).wref();
                        return EigenVectorMap(bvec.data()->ptr(), sofa::defaulttype::Vec3Types::Coord::size() * bvec.size());
                    }
                    else
                        std::cout << "Python error in LinearSolver x() :\nWe cannot find any MechanicalObject from LinearSolver context and childs, we cannot get x" << std::endl;
                }
                else
                    std::cout << "Python error in LinearSolver x() :\nInput parameter is not a MState link (interpreted as py::str)" << std::endl;
            }
            return {};
        }, sofapython3::doc::linearsolver::linearSolver_b);


    c.def("b", [](GSLinearSolver& self, py::object& mstateLink) -> Vector
    {
        if (const GSV* vector = self.getSystemRHVector())
        {
            if (py::isinstance<py::str>(mstateLink))
            {
                sofa::core::behavior::MechanicalState<sofa::defaulttype::Rigid3Types>* rmstate = nullptr;
                self.getContext()->findLinkDest(rmstate, mstateLink.cast<std::string>(), nullptr);
                if (rmstate)
                {
                    auto bvec = sofa::helper::getWriteAccessor(*vector->id()[rmstate].write()).wref();
                    return EigenVectorMap(bvec.data()->ptr(), sofa::defaulttype::Rigid3Types::Deriv::size() * bvec.size());
                }
                sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3Types>* mstate = nullptr;
                self.getContext()->findLinkDest(mstate, mstateLink.cast<std::string>(), nullptr);
                if (mstate)
                {
                    auto bvec = sofa::helper::getWriteAccessor(*vector->id()[mstate].write()).wref();
                    return EigenVectorMap(bvec.data()->ptr(), sofa::defaulttype::Vec3Types::Coord::size() * bvec.size());
                }
                else
                    std::cout << "Python error in LinearSolver b() :\nWe cannot find any MechanicalObject from LinearSolver context and childs, we cannot get b" << std::endl;
            }
            else
                std::cout << "Python error in LinearSolver b() :\nInput parameter is not a MState link (interpreted as py::str)" << std::endl;
        }
        return {};
    }, sofapython3::doc::linearsolver::linearSolver_b);



    c.def("setSystemLHVector", [](GSLinearSolver& self, py::object& mstateLink, py::object vecObject)
    {
        if (GSV* vecLH = self.getSystemLHVector())
        {
            auto vec = py::cast<py::array_t<double, py::array::c_style | py::array::forcecast>>(vecObject).unchecked<1>();
            int vId = 0;

            if (py::isinstance<py::str>(mstateLink))
            {
                sofa::core::behavior::MechanicalState<sofa::defaulttype::Rigid3Types>* rmstate = nullptr;
                self.getContext()->findLinkDest(rmstate, mstateLink.cast<std::string>(), nullptr);
                if (rmstate)
                {
                    auto xvec = sofa::helper::getWriteAccessor(*vecLH->id()[rmstate].write());
                    if (vec.size() >= (3 * xvec.size()))
                    {
                        for (size_t i = 0; i < xvec.size(); i++)
                        {
                            xvec[i] = sofa::defaulttype::Rigid3dTypes::Deriv(sofa::type::Vec3d(vec(vId), vec(vId + 1), vec(vId + 2)), sofa::type::Vec3d(vec(vId + 3), vec(vId + 4), vec(vId + 5)));
                            vId += 6;
                        }
                        return true;
                    }
                    else
                        std::cerr << "Python error in LinearSolver setSystemLHVector() :\nSize mismatch between X (in mechanicalstates " << mstateLink.cast<std::string>() << ", =" << xvec.size() << ") and input vector (" << vec.size() << ")" << std::endl;
                }
                sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3Types>* mstate = nullptr;
                self.getContext()->findLinkDest(mstate, mstateLink.cast<std::string>(), nullptr);
                if (mstate)
                {
                    auto xvec = sofa::helper::getWriteAccessor(*vecLH->id()[mstate].write());
                    if (vec.size() >= (3 * xvec.size()))
                    {
                        for (size_t i = 0; i < xvec.size(); i++)
                        {
                            xvec[i] = sofa::defaulttype::Vec3dTypes::Coord(vec(vId), vec(vId + 1), vec(vId + 2));
                            vId += 6;
                        }
                        return true;
                    }
                    else
                        std::cerr << "Python error in LinearSolver setSystemLHVector() :\nSize mismatch between X (in mechanicalstates " << mstateLink.cast<std::string>() << ", =" << xvec.size() << ") and input vector (" << vec.size() << ")" << std::endl;
                }
            }
        }

        return false;
    });

    c.def("setSystemRHVector", [](GSLinearSolver& self, py::object& mstateLink, py::object vecObject)
    {
        if (GSV* vecRH = self.getSystemRHVector())
        {
            auto vec = py::cast<py::array_t<double, py::array::c_style | py::array::forcecast>>(vecObject).unchecked<1>();
            int vId = 0;

            if (py::isinstance<py::str>(mstateLink))
            {
                sofa::core::behavior::MechanicalState<sofa::defaulttype::Rigid3Types>* rmstate = nullptr;
                self.getContext()->findLinkDest(rmstate, mstateLink.cast<std::string>(), nullptr);
                if (rmstate)
                {
                    auto xvec = sofa::helper::getWriteAccessor(*vecRH->id()[rmstate].write());
                    if (vec.size() >= (3 * xvec.size()))
                    {
                        for (size_t i = 0; i < xvec.size(); i++)
                        {
                            xvec[i] = sofa::defaulttype::Rigid3dTypes::Deriv(sofa::type::Vec3d(vec(vId), vec(vId + 1), vec(vId + 2)), sofa::type::Vec3d(vec(vId + 3), vec(vId + 4), vec(vId + 5)));
                            vId += 6;
                        }
                        return true;
                    }
                    else
                        std::cerr << "Python error in LinearSolver setSystemLHVector() :\nSize mismatch between X (in mechanicalstates " << mstateLink.cast<std::string>() << ", =" << xvec.size() << ") and input vector (" << vec.size() << ")" << std::endl;
                }
                sofa::core::behavior::MechanicalState<sofa::defaulttype::Vec3Types>* mstate = nullptr;
                self.getContext()->findLinkDest(mstate, mstateLink.cast<std::string>(), nullptr);
                if (mstate)
                {
                    auto xvec = sofa::helper::getWriteAccessor(*vecRH->id()[mstate].write());
                    if (vec.size() >= (3 * xvec.size()))
                    {
                        for (size_t i = 0; i < xvec.size(); i++)
                        {
                            xvec[i] = sofa::defaulttype::Vec3dTypes::Coord(vec(vId), vec(vId + 1), vec(vId + 2));
                            vId += 6;
                        }
                        return true;
                    }
                    else
                        std::cerr << "Python error in LinearSolver setSystemLHVector() :\nSize mismatch between X (in mechanicalstates " << mstateLink.cast<std::string>() << ", =" << xvec.size() << ") and input vector (" << vec.size() << ")" << std::endl;
                }
            }

        }
        return false;
    });

    c.def("solveSystem", py::overload_cast<>(&GSLinearSolver::solveSystem), sofapython3::doc::linearsolver::linearSolver_solveSystem);

    /// register the binding in the downcasting subsystem
    PythonFactory::registerType<GSLinearSolver>([](sofa::core::objectmodel::Base* object)
        {
            return py::cast(dynamic_cast<GSLinearSolver*>(object));
        });
}


void moduleAddLinearSolver(py::module &m)
{
    bindLinearSolvers<SReal>(m);
    bindLinearSolvers<sofa::type::Mat<3, 3, SReal> >(m);

    bindMatrixFreeLinearSolvers(m);
}

}
