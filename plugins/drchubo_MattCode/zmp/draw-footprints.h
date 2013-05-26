#ifndef DRAW_FOOTPRINTS_H
#define DRAW_FOOTPRINTS_H

#include "mzcommon/MzGlutApp.h"
#include <vector>
#include "zmp/footprint.h"

/**
 * \fn sampleFootprints
 * \brief Generates a vector of simple footprints in a straight line, slightly turned out
 * \param [in] numPrints The number of feet to draw
 */
std::vector<Footprint> sampleFootprints(int numPrints)
{
	std::vector<Footprint> prints;
	for (int i = 0; i < numPrints; i++)
	{
		if (i%2==0)
		{
			prints.push_back(Footprint(0.2*i,-0.1,0.1,true));
		}
		else
		{
			prints.push_back(Footprint(0.2*i,0.1,-0.1,false));
		}

	}
	return prints;
}

/**
 * \fn drawFootprints
 * \brief Draws a vector of footprints at z=0 in the current OpenGL context
 * \param [in] footprints The vector containing the footprints to draw
 * \param [in] foot_width Sets the width of the drawn foot's bounding box. The
 * height is twice the width.
 */
void drawFootprints(const std::vector<Footprint>& footprints, const double foot_width = 0.1)
{
	double foot_bounds[4][2] = {
			{foot_width,foot_width/2},
			{foot_width,-foot_width/2},
			{-foot_width,-foot_width/2},
			{-foot_width,foot_width/2},};

	for (int i = 0; i < (int)footprints.size(); i++)
	{
		// Pick the color for left/right
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		if (footprints[i].is_left)
		{
			glColor3ub(0,0,255);
		}
		else
		{
			glColor3ub(255,0,0);
		}

		// Transform into the foot's frame
		glPushMatrix();
		glTranslated(footprints[i].x, footprints[i].y, 0.0);
		glRotated(-footprints[i].theta*180/M_PI, 0.0, 0.0, 1.0); //in degrees ?!

		// Draw the center of the foot
		glBegin(GL_POINTS);
		glVertex3d(0.0, 0.0, 0.0);
		glEnd( );

		// Draw foot's bounding box (decoration)
		glBegin(GL_LINES);
		for (int j = 0; j < 4; j++)
		{
			int j2 = (j+1) % 4;
			glVertex3d(foot_bounds[j][0], foot_bounds[j][1], 0.0);
			glVertex3d(foot_bounds[j2][0], foot_bounds[j2][1], 0.0);
		}
		glEnd( );
		glPopMatrix();
	}
}

#endif // DRAW_FOOTPRINTS_H
