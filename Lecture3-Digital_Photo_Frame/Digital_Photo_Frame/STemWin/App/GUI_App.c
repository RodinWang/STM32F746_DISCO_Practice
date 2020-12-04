  /**
  ******************************************************************************
  * @file    GUI_App.c
  * @author  MCD Application Team
  * @brief   Simple demo drawing "Hello world"  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright © 2018 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
#include "GUI_App.h"
#include "GUI.h"
#include "fatfs.h"
#include "string.h"
#include "stdio.h"

// Display Mode Play or Stop
DisplayMode_Typedef DisplayMode;

// previous Photo update time
GUI_TIMER_TIME PrvUpdateTime;
GUI_TIMER_TIME NowTime;
// File system
uint32_t MaxFileCount = 0;
TCHAR FileNameTable[256][50];

// image 
GUI_HMEM hMem;


// Scan File
uint32_t ScanBMPFiles (const TCHAR* DirName){
	FRESULT f_res; 
	DIR f_dir; 
	FILINFO f_info;
  uint32_t index = 0;
  /* Open filesystem */

  /* Start to search for wave files */
  f_res = f_findfirst(&f_dir, &f_info, (DirName), _T("*.bmp"));

  /* Repeat while an item is found */
  while (f_info.fname[0])
  {
    if(f_res == FR_OK)
    {
      if(index < 256)
      {
				memcpy(FileNameTable[index], f_info.fname, 50*sizeof(TCHAR));
				index++;
			}
			else{
				break;
			}
      /* Search for next item */
      f_res = f_findnext(&f_dir, &f_info);
    }
    else
    {
      index = 0;
      break;
    }
  }

  f_closedir(&f_dir);
  return index;
}

uint8_t* LoadBMP(uint32_t FileIndex){
	FRESULT f_res;
	uint8_t* ImageBuffer;
	FSIZE_t ImageSize;
	UINT ReadSize;
	
	if (FileIndex >= MaxFileCount){
			GUI_DispStringAt("Image Number Out of range", 0, 0);
			while(1);
	}
	
	// open file
	f_res = f_open(&SDFile, FileNameTable[FileIndex], FA_READ);
	if(f_res != FR_OK){
			GUI_DispStringAt("Open File Error!", 0, 0);
			return NULL;
	}
	
	// Malloc
	ImageSize = SDFile.obj.objsize;
	hMem = GUI_ALLOC_AllocZero(ImageSize);
	ImageBuffer = GUI_ALLOC_h2p(hMem);
	
	// Read File
	f_res = f_read(&SDFile, ImageBuffer, ImageSize, &ReadSize);
	if (f_res != FR_OK){
		GUI_DispStringAt("Read File Error!", 0, 0);
		f_close(&SDFile);
		GUI_ALLOC_Free(hMem);
		return NULL;
	}
	f_close(&SDFile);
	
	return ImageBuffer;	
}

// display image
void DisplayLoadedImage(uint8_t *ImageBuffer){
	uint32_t ImageSizeX, ImageSizeY;
	uint32_t ImageScaleFraction, ImageScaleDenominator;
	
	// Get Image Size
	ImageSizeX = GUI_BMP_GetXSize(ImageBuffer);
	ImageSizeY = GUI_BMP_GetYSize(ImageBuffer);
	ImageScaleFraction = ImageSizeX;
	ImageScaleDenominator = ImageSizeX;
	
	// get image scale ratio
	if ( ((float)ImageSizeX/(float)LCD_GetXSize()) >= ((float)ImageSizeY/(float)LCD_GetYSize())){
		ImageScaleFraction = LCD_GetXSize();
		ImageScaleDenominator = ImageSizeX;
	}
	else{
		ImageScaleFraction = LCD_GetYSize();
		ImageScaleDenominator = ImageSizeY;
	}
	// clear and display new image
	//GUI_ClearRect(0, 0, LCD_GetXSize(), LCD_GetYSize());
	GUI_Clear();
	GUI_BMP_DrawScaled(
		ImageBuffer, 
		(LCD_GetXSize() - ImageSizeX * ImageScaleFraction / ImageScaleDenominator)/2,
		(LCD_GetYSize() - ImageSizeY * ImageScaleFraction / ImageScaleDenominator)/2, 
		ImageScaleFraction, 
		ImageScaleDenominator
	);
	// free loaded image
	GUI_ALLOC_Free(hMem);
}

void GRAPHICS_MainTask(void) {

/* USER CODE BEGIN GRAPHICS_MainTask */
 /* User can implement his graphic application here */
  uint8_t *ImageBuffer;
	uint32_t CurrentFileIndex = 0;
	uint32_t LoadedFileIndex = 255;
	
	// Scan for BMP file
	MaxFileCount = ScanBMPFiles(_T(""));
	if(MaxFileCount == 0){
		GUI_DispStringAt("BMP File Not Found!!", 0, 0);
		while(1);
	}
	
	// Load BMP file
	if (LoadedFileIndex != CurrentFileIndex){
		ImageBuffer = LoadBMP(CurrentFileIndex);
		if (ImageBuffer == NULL) {
			GUI_DispStringAt("BMP File Load Failed!!", 0, 0);
			while(1);
		}
		LoadedFileIndex = CurrentFileIndex;
	}
	// Display Image
	DisplayLoadedImage(ImageBuffer);
	
	// Next Image
	CurrentFileIndex = (LoadedFileIndex + 1) % MaxFileCount;
	
/* USER CODE END GRAPHICS_MainTask */
  while(1)
	{
		// Play Mode
		if (DisplayMode == PLAY_MODE){
				NowTime = GUI_GetTime();
				// Last update time larger than 4s
				if (NowTime - PrvUpdateTime >= 4000){
						DisplayLoadedImage(ImageBuffer);
						PrvUpdateTime = GUI_GetTime();
						// next picture
						if (LoadedFileIndex == CurrentFileIndex)	
							CurrentFileIndex = (LoadedFileIndex + 1) % MaxFileCount;
				}
		}
		
		// Load Next Image
		if (LoadedFileIndex != CurrentFileIndex){
				ImageBuffer = LoadBMP(CurrentFileIndex);	
				LoadedFileIndex = CurrentFileIndex;			
		}
		GUI_Delay(10);
	}
}

/*************************** End of file ****************************/
