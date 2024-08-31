#include "MatrixStack.h"

MatrixStack::MatrixStack()
{
    m_matrices.push_back(Matrix4f::identity());
}

void MatrixStack::clear()
{
    m_matrices.clear();
    m_matrices.push_back(Matrix4f::identity());
}

Matrix4f MatrixStack::top()
{
	// Return the top of the stack
	return m_matrices.back();
}

void MatrixStack::push( const Matrix4f& m )
{
	// Push m onto the stack.
	// Your stack should have OpenGL semantics:
	// the new top should be the old top multiplied by m
    Matrix4f temp = m_matrices.back() * m;
    m_matrices.push_back(temp);
}

void MatrixStack::pop()
{
    m_matrices.pop_back();
}
