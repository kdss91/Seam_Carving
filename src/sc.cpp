
#include "sc.h"

using namespace cv;
using namespace std;




bool seam_carving(Mat& in_image, int new_width, int new_height, Mat& out_image){

    // some sanity checks
    // Check 1 -> new_width <= in_image.cols
    if(new_width>in_image.cols){
        cout<<"Invalid request!!! new_width has to be smaller than the current size!"<<endl;
        return false;
    }
    if(new_height>in_image.rows){
        cout<<"Invalid request!!! ne_height has to be smaller than the current size!"<<endl;
        return false;
    }
    
    if(new_width<=0){
        cout<<"Invalid request!!! new_width has to be positive!"<<endl;
        return false;

    }
    
    if(new_height<=0){
        cout<<"Invalid request!!! new_height has to be positive!"<<endl;
        return false;
        
    }

    
    return seam_carving_trivial(in_image, new_width, new_height, out_image);
}


// seam carves by removing trivial seams
bool seam_carving_trivial(Mat& in_image, int new_width, int new_height, Mat& out_image){

    Mat iimage = in_image.clone();
    Mat oimage = in_image.clone();
    while(iimage.rows!=new_height || iimage.cols!=new_width){
        // horizontal seam if needed
        if(iimage.rows>new_height){
            reduce_horizontal_seam_trivial(iimage, oimage);
            iimage = oimage.clone();
        }
        
        if(iimage.cols>new_width){
            reduce_vertical_seam_trivial(iimage, oimage);
            iimage = oimage.clone();
        }
    }
    
    out_image = oimage.clone();
    return true;
}

// horizontl trivial seam is a seam through the center of the image
bool reduce_horizontal_seam_trivial(Mat& in_image, Mat& out_image){

    // retrieve the dimensions of the new image
    int rows = in_image.rows;
    int cols = in_image.cols;

    // create an image slighly smaller
    out_image = Mat(rows-1, cols, CV_8UC3);

    Mat src, src_gray;
    Mat gradient_matrix;
    
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    src = in_image.clone();
    
    GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
 
    Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );
    convertScaleAbs( grad_y, abs_grad_y );
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, gradient_matrix );


    Mat diff_mat(rows, cols, CV_32S);

    

    for(int i=0;i<rows;++i){
        diff_mat.at<int>(i,0) = abs((int)gradient_matrix.at<char>(i,0));
    }

    int countR=0,countC=1;


    while(countC<cols){    
        countR=0;
        while(countR<rows){
            int var1, var2, var3;  
            if(countR==rows-1){
                var1 = abs((double)gradient_matrix.at<char>(countR,countC) - (double)gradient_matrix.at<char>(countR,countC-1)) + diff_mat.at<int>(countR,countC-1);
                var2 = abs((double) gradient_matrix.at<char>(countR,countC) - (double)gradient_matrix.at<char>(countR-1,countC-1)) + diff_mat.at<int>(countR-1,countC-1);
                diff_mat.at<int>(countR,countC) = (var1<=var2)?var1:var2;
            }
            else if(countR==0){
                var1 = abs((double)gradient_matrix.at<char>(countR,countC) - (double)gradient_matrix.at<char>(countR,countC-1)) + diff_mat.at<int>(countR,countC-1);
                var2 = abs((double) gradient_matrix.at<char>(countR,countC) - (double)gradient_matrix.at<char>(countR+1,countC-1)) + diff_mat.at<int>(countR+1,countC-1);
                diff_mat.at<int>(countR,countC) = (var1<=var2)?var1:var2;
            }
            else {
                var1 = abs((double)gradient_matrix.at<char>(countR,countC) - (double)gradient_matrix.at<char>(countR+1,countC-1)) + diff_mat.at<int>(countR+1,countC-1);
                var2 = abs((double)gradient_matrix.at<char>(countR,countC) - (double)gradient_matrix.at<char>(countR,countC-1)) + diff_mat.at<int>(countR,countC-1);
                var3 = abs((double)gradient_matrix.at<char>(countR,countC) - (double)gradient_matrix.at<char>(countR-1,countC-1)) + diff_mat.at<int>(countR-1,countC-1);
                diff_mat.at<int>(countR,countC) = (((var1<=var2)?var1:var2)<=var3)?((var1<=var2)?var1:var2):var3;
            }

            countR++;

        }
        countC++;
    }


    int minimum=INT_MAX;
    int ymin=0;

    for(int i=0;i<rows;++i){
        if(diff_mat.at<int>(i,cols-1) < minimum){
            minimum = diff_mat.at<int>(i,cols-1);
            ymin=i;
        }
    }


    int lastYPos = ymin;
    int path [cols];

    int countX = cols-1;
    path[countX]=lastYPos;

    while(countX>0){
            countX--;
            if(lastYPos==rows-1){
                if(diff_mat.at<int>(lastYPos, countX) >= diff_mat.at<int>(lastYPos-1, countX)) {
                    lastYPos=lastYPos-1;
                }
            }

            else if(lastYPos==0){
                if(diff_mat.at<int>(lastYPos, countX) >= diff_mat.at<int>(lastYPos+1, countX)) {
                    lastYPos=lastYPos+1;
                }
            }
           
            else {
                if(diff_mat.at<int>(lastYPos, countX) >= diff_mat.at<int>(lastYPos-1, countX) && diff_mat.at<int>(lastYPos+1, countX) >= diff_mat.at<int>(lastYPos-1, countX)) {
                    lastYPos=lastYPos-1;
                }
                else if(diff_mat.at<int>(lastYPos, countX) >= diff_mat.at<int>(lastYPos+1, countX) && diff_mat.at<int>(lastYPos-1, countX) >= diff_mat.at<int>(lastYPos+1, countX)) {
                    lastYPos=lastYPos+1;
                }
            }
            path[countX]=lastYPos;
    }



    int count=0;

    while(count<cols){
            int yPos = path[count];

            if(yPos==rows-1) {
                for(int k=0;k<rows-1;++k){
                    out_image.at<Vec3b>(k, count) = in_image.at<Vec3b>(k, count); 
                }
            }

            else if(yPos==0){
                for(int k=1;k<rows;++k){
                    out_image.at<Vec3b>(k-1, count) = in_image.at<Vec3b>(k, count); 
                }
            }

            else {
                for(int k=0;k<yPos;++k){
                    out_image.at<Vec3b>(k, count) = in_image.at<Vec3b>(k, count); 
                }
                for(int k=yPos+1;k<rows;++k){
                    out_image.at<Vec3b>(k-1, count) = in_image.at<Vec3b>(k, count); 
                }
            }

            count++;
    }

    return true;

}

// vertical trivial seam is a seam through the center of the image
bool reduce_vertical_seam_trivial(Mat& in_image, Mat& out_image){


    // retrieve the dimensions of the new image
    int rows = in_image.rows;
    int cols = in_image.cols;

    // create an image slighly smaller
    out_image = Mat(rows, cols-1, CV_8UC3);

    Mat src, src_gray;
    Mat gradient_matrix;
    
    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;
    src = in_image.clone();
    
    GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;
 
    Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
    Sobel( src_gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_x, abs_grad_x );
    convertScaleAbs( grad_y, abs_grad_y );
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, gradient_matrix );


    Mat diff_mat(rows, cols, CV_32S);
    

    for(int j=0;j<cols;++j){
        diff_mat.at<int>(rows-1,j) = abs((int) gradient_matrix.at<char>(rows-1,j));
    }

    int countR=rows-2,countC=0;

    while(countR>=0){
        countC=0;
        while(countC<cols){
            int var1, var2, var3;
            if(countC==cols-1){
                var1 = abs(gradient_matrix.at<char>(countR,countC) - gradient_matrix.at<char>(countR+1,countC)) + diff_mat.at<int>(countR+1,countC);
                var2 = abs(gradient_matrix.at<char>(countR,countC) - gradient_matrix.at<char>(countR+1,countC-1)) + diff_mat.at<int>(countR+1,countC-1);
                diff_mat.at<int>(countR,countC) = (var1<=var2)?var1:var2;
            }
            else if(countC==0){
                var1 = abs(gradient_matrix.at<char>(countR,countC) - gradient_matrix.at<char>(countR+1,countC)) + diff_mat.at<int>(countR+1,countC);
                var2 = abs(gradient_matrix.at<char>(countR,countC) - gradient_matrix.at<char>(countR+1,countC+1)) + diff_mat.at<int>(countR+1,countC+1);
                diff_mat.at<int>(countR,countC) = (var1<=var2)?var1:var2;
            }
            else{
                var1 = abs(gradient_matrix.at<char>(countR,countC) - gradient_matrix.at<char>(countR+1,countC-1)) + diff_mat.at<int>(countR+1,countC-1);
                var2 = abs(gradient_matrix.at<char>(countR,countC) - gradient_matrix.at<char>(countR+1,countC)) + diff_mat.at<int>(countR+1,countC);
                var3 = abs(gradient_matrix.at<char>(countR,countC) - gradient_matrix.at<char>(countR+1,countC+1)) + diff_mat.at<int>(countR+1,countC+1);
                diff_mat.at<int>(countR,countC) = (((var1<=var2)?var1:var2)<=var3)?((var1<=var2)?var1:var2):var3;
            }
            countC++;
        }
        countR--;
    }


    int minimum=INT_MAX;
    int xmin=0;

    for(int j=0;j<cols;++j){
        if(diff_mat.at<int>(0,j) < minimum){
            minimum = diff_mat.at<int>(0,j);
            xmin = j;
        }
    }

    int lastXPos = xmin;
    int path [rows];

    int countY = 0;
    path[countY]=lastXPos;

    while(countY<rows-1){
            countY++;
             
            if(lastXPos==cols-1){
                if(diff_mat.at<int>(countY,lastXPos) >= diff_mat.at<int>(countY,lastXPos-1)) {
                    lastXPos=lastXPos-1;
                }
            }

            else if(lastXPos==0){
                if(diff_mat.at<int>(countY,lastXPos) >= diff_mat.at<int>(countY,lastXPos+1)) {
                    lastXPos=lastXPos+1;
                }
            }
          
            else {
                if(diff_mat.at<int>(countY,lastXPos) >= diff_mat.at<int>(countY,lastXPos-1) && diff_mat.at<int>(countY,lastXPos+1) >= diff_mat.at<int>(countY,lastXPos-1)) {
                    lastXPos=lastXPos-1;
                }
                else if(diff_mat.at<int>(countY,lastXPos) >= diff_mat.at<int>(countY,lastXPos+1) && diff_mat.at<int>(countY,lastXPos-1) >= diff_mat.at<int>(countY,lastXPos+1)) {
                    lastXPos=lastXPos+1;
                }
            }

            path[countY]=lastXPos;
    }


    int count=0;

    while(count<rows){
            int xPos = path[count];
        
            if(xPos==cols-1) {
                for(int k=0;k<cols-1;++k){
                    out_image.at<Vec3b>(count, k) = in_image.at<Vec3b>(count, k); 
                }
            }

            else if(xPos==0){
                for(int k=1;k<cols;++k){
                    out_image.at<Vec3b>(count, k-1) = in_image.at<Vec3b>(count, k); 
                }
            }

            else {
                for(int k=0;k<xPos;++k){
                    out_image.at<Vec3b>(count, k) = in_image.at<Vec3b>(count, k); 
                }
                for(int k=xPos+1;k<cols;++k){
                    out_image.at<Vec3b>(count, k-1) = in_image.at<Vec3b>(count, k); 
                }
            }

            count++;
    }

    return true;
}

