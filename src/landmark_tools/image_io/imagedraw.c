/** 
 * \copyright Copyright 2024 California Institute of Technology
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include <math.h>

#include "landmark_tools/image_io/imagedraw.h"
#include "landmark_tools/math/math_constants.h"  // for PI

int32_t DrawFeatureBlock (uint8_t *pixels, int32_t cols, int32_t rows, double x, double y, uint8_t v, int32_t size)
{
	int32_t i,j;
	int32_t k;
	int32_t minx, maxx, miny, maxy;


	if (size == 1) 
		k = 0;
	else
		k = size/2;
	
	minx = (int32_t)(x - (double)k);
	maxx = (int32_t)(x + (double)k);
	miny = (int32_t)(y - (double)k);
	maxy = (int32_t)(y + (double)k);


	if (size == 2)
	{
		maxx--;
		maxy--;
	}
	if (minx<0) minx=0;
	if (minx>cols) minx=cols-1;
	if (miny<0) miny=0;
	if (miny>rows) miny=rows-1;
	if (maxx<0) maxx=0;
	if (maxx>cols) maxx=cols-1;
	if (maxy<0) maxy=0;
	if (maxy>rows) maxy=rows-1;

	for (j=miny; j<=maxy; j++)
	  for (i=minx; i<= maxx; i++)
		pixels[i+j*cols]=v;
   
return 1;
}

int32_t DrawFeatureCircle (uint8_t *pixels, int32_t cols, int32_t rows, double x, double y, uint8_t v, int32_t size)
{
	int32_t i,j;
	int32_t k;
	int32_t minx, maxx, miny, maxy;



	k = 1;
	minx = (int32_t)(x - (double)k);
	maxx = (int32_t)(x + (double)k);
	miny = (int32_t)(y - (double)k);
	maxy = (int32_t)(y + (double)k);
	
	if (size == 2)
	{
		maxx--;
		maxy--;
	}
	if (minx<0) minx=0;
	if (minx>cols) minx=cols-1;
	if (miny<0) miny=0;
	if (miny>rows) miny=rows-1;
	if (maxx<0) maxx=0;
	if (maxx>cols) maxx=cols-1;
	if (maxy<0) maxy=0;
	if (maxy>rows) maxy=rows-1;

	for (j=miny; j<=maxy; j++)
	  for (i=minx; i<= maxx; i++)
		pixels[i+j*cols]=v;

    DrawCircle(pixels,  cols,rows,  x, y, (double)size, v); 
    DrawCircle(pixels,  cols,rows,  x, y, (double)(size+1), v);
    return 1;
}

int32_t DrawFeatureCross (uint8_t *pixel, int32_t cols, int32_t rows, double x, double y, uint8_t v, int32_t size)
{
	int32_t k;
	
	k = size/2;
    DrawLine(pixel, cols,rows,x-k, y, x+k, y, v, 1);
	DrawLine(pixel, cols,rows,x, y-k, x, y+k, v, 1);
	return 1;
}

int32_t DrawArrow(uint8_t *greyscale, int32_t cols, int32_t rows, double x0, double y0, double  x1, double  y1, uint8_t c, int32_t size)
{
	 
	double x, y, d;
	double dx1, dy1;
	double dx, dy;
	dx = 7*cos(3.14/3.0);
    dy = 7*sin(3.14/3.0);
    
	DrawLine(greyscale, cols, rows, x0, y0, x1, y1, c, size);
	
	x = x1 - x0;
	y = y1 - y0;
	d = sqrt(x*x + y*y);
	x = x/d;
	y = -y/d;
    dx1 = x1 - dx*y - dy*x + 0.5;
	dy1 = y1 - dx*x + dy*y +0.5; 
	DrawLine(greyscale, cols, rows, x1, y1, dx1, dy1, c, size);
	
	dx1 = x1 + dx*y - dy*x;
	dy1 = y1 + dx*x + dy*y; 
	DrawLine(greyscale, cols, rows, x1, y1, dx1, dy1, c, size);
	return 1;
}

int32_t DrawLine(uint8_t *greyscale, int32_t cols, int32_t rows, double x0, double y0, double  x1, double  y1, uint8_t c, int32_t size)
{
   int32_t x,y,i,incx,incy,sx,sy,e, j;
   
   int32_t hs;
   
   hs = size/2;
   //if(size == 1) hs = 1;

   sx=(int16_t)fabs((double)(x1-x0));
   sy=(int16_t)fabs((double)(y1-y0));
   if ((y1-y0)>0) incy=1;
   else incy= -1;
   if ((x1-x0)>0) incx=1;
   else incx= -1;
   if (sx>sy)
   {
        e=2*sy-sx;
        y=(int32_t)y0;
		x=(int32_t)x0;

        for(i=1;i<=sx;i++)
        {
            if (x>hs && x<cols-hs && y> hs &&  y < rows-hs)
			{
                for(j = -hs; j <= hs; ++j)
				{
					greyscale[(y+j)*cols + x] = c;
				}
			}
            x+=incx;
            if (e<0)
                e+=2*sy;
            else
            {
                y+=incy;
                e+=2*sy-2*sx;
            }
        }
    }
    else
    {
        e=2*sx-sy;
        x= (int32_t)x0;
		y= (int32_t)y0;
        for(i=1;i<=sy;i++)
        {
            if (x>hs && x<cols-hs && y> hs &&  y < rows-hs)
			{
				for(j = -hs; j <= hs; ++j)
				{
					greyscale[y*cols + x + j] = c;
				}
			}
            
            y+=incy;
            if (e<0)
                e+=2*sx;
            else
            {
                x+=incx;
                e+=2*sx-2*sy;
            }
        }
    }
    return 1;
}

int32_t DrawEllipse(uint8_t *greyscale, int32_t cols,int32_t rows,  double x0, double y0, double a, double b,
                      double theta, uint8_t c)
{
    int32_t i,xa1,ya1,xb1,yb1,max;
    double xa0,ya0,xb0,yb0,step;
	max = (int32_t)(a*b*2);
	step=2*PI/(double)max;

    for(i=0;i<max;i++)
    {
        xa0=a*cos(i*step);
        ya0=b*sin(i*step);
        xb0=a*cos(((i+1)%(max))*step);
        yb0=b*sin(((i+1)%(max))*step);
        xa1=(int32_t)(xa0*cos(theta)-ya0*sin(theta)+x0+0.5);
        ya1=(int32_t)(xa0*sin(theta)+ya0*cos(theta)+y0+0.5);
        xb1=(int32_t)(xb0*cos(theta)-yb0*sin(theta)+x0+0.5);
        yb1=(int32_t)(xb0*sin(theta)+yb0*cos(theta)+y0+0.5);
		if(xa1 > 0 && xa1< cols && ya1 > 0 && ya1 < rows &&
		   xb1 > 0 && xb1 < cols && yb1 > 0 && yb1 < rows)
		{
			DrawLine(greyscale, cols, rows, xa1,ya1,xb1,yb1,c, 1);  
		}
	}
    return 1;
}


int32_t FillEllipse(uint8_t *greyscale, int32_t cols,int32_t rows,  double x0, double y0, double a, double b,
                      double theta, uint8_t c)
{
    int32_t xa1,ya1,xb1,yb1,max=(int32_t)(a*b*2);
    double xa0,ya0,xb0,yb0,step=2*PI/(double)max;
	int32_t xmin, ymin, xmax, ymax;
	int32_t i, j;
	int32_t xl, xr;

    xmin = cols;
	xmax = 0;
	ymin = rows;
	ymax = 0;
    for(i=0;i<max;i++)
    {
        xa0=a*cos(i*step);
        ya0=b*sin(i*step);
        xb0=a*cos(((i+1)%(max))*step);
        yb0=b*sin(((i+1)%(max))*step);

        xa1=(int32_t)(xa0*cos(theta)-ya0*sin(theta)+x0+0.5);
        ya1=(int32_t)(xa0*sin(theta)+ya0*cos(theta)+y0+0.5);
        xb1=(int32_t)(xb0*cos(theta)-yb0*sin(theta)+x0+0.5);
        yb1=(int32_t)(xb0*sin(theta)+yb0*cos(theta)+y0+0.5);
		if(xa1 > 0 && xa1< cols && ya1 > 0 && ya1 < rows &&
		   xb1 > 0 && xb1 < cols && yb1 > 0 && yb1 < rows)
		{
			DrawLine(greyscale, cols, rows, xa1,ya1,xb1,yb1,c-1, 1);  
			if(xa1 < xmin) xmin = xa1;
            if(xa1 > xmax) xmax = xa1;
			if(ya1 < ymin) ymin = ya1;
			if(ya1 > ymax) ymax = ya1;
		}
		else if(xa1 <=0)
		{
            greyscale[ya1*cols] = c-1;
			xmin = 0;
		}
        else if(xb1 <=0)
		{
            greyscale[yb1*cols] = c-1;
			xmin = 0;
		}
        else if(xa1 >= cols)
		{
            greyscale[ya1*cols + cols-1] = c-1;
			xmax = cols-1;
		}
        else if(xb1 >= cols)
		{
            greyscale[yb1*cols + cols-1] = c-1;
			xmax = cols -1;
		}

	}
	for(i = ymin; i <= ymax; ++i)
	{
		for(j = xmin, xl = xmin; j <= xmax; ++j)
		{
			if(greyscale[i*cols + j] == c-1)
			{
				xl = j;
				break;
			}
		}
        for(j = xmax, xr = xmax; j >= xmin; --j)
		{
			if(greyscale[i*cols + j] == c-1)
			{
				xr = j;
				break;
			}
		}
		for(j = xl; j <= xr; ++j)
		{
           greyscale[i*cols + j] = c;
		}
	}
    return 1;
}


int32_t DrawBox(uint8_t *greyscale, int32_t cols,int32_t rows,  double x0, double y0, int32_t boxsize, 
                       uint8_t c, int32_t linesize)
{
   int32_t x1, y1, x2, y2;
   y2 = (int32_t)(y0 - (double)(boxsize/2));
   y1 = y2;
   x1 = (int32_t)(x0 - (double)(boxsize/2));
   x2 = (int32_t)(x0 + (double)(boxsize/2));
   DrawLine(greyscale, cols, rows, x1, y1, x2, y2, c, linesize);
   y2 = y2+boxsize;
   y1 = y1+boxsize;
   DrawLine(greyscale, cols, rows, x1, y1, x2, y2, c, linesize);
   x2 = (int32_t)(x0 - (double)(boxsize/2));
   x1 = x2;
   y1 = (int32_t)(y0 - (double)(boxsize/2));
   y2 = (int32_t)(y0 + (double)(boxsize/2));
   DrawLine(greyscale, cols, rows, x1, y1, x2, y2, c, linesize);
   x2 = x2+boxsize;
   x1 = x1+boxsize;
   DrawLine(greyscale, cols, rows, x1, y1, x2, y2, c, linesize);
   return 1;

}

int32_t DrawCircle(uint8_t *greyscale, int32_t cols,int32_t rows,  double x0, double y0, double a, 
                       uint8_t c)
{
   DrawEllipse(greyscale, cols, rows, x0,  y0,  a, a, 0.0, c);
                      
   return 1;
}
