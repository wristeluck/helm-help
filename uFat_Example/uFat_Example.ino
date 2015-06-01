#include <Arduino.h>
#include <avr/pgmspace.h>
#include <microfat2.h>
#include <mmc.h>

byte sector_buffer[512];

char sprint_buffer[40];

// BEWARE - don't print strings longer than 39 characters! 
//          If you can't help it, adjust buffer size above.
//
void pprint(const char* s)
{
  strcpy_P(sprint_buffer, (PGM_P)s);
  Serial.print(sprint_buffer);
}

void error(const char* s)
{
  pprint(PSTR("Error: "));
  pprint(s);
  pprint(PSTR("<press reset>"));
  for( /* ever */ ; ; ) 
  {
    digitalWrite(13, (millis() / 250) & 1);
  }
}

void file_details(const char *filename) {
  unsigned long sector;
  unsigned long byteSize;

  if (microfat2::locateFileStart(filename, sector, byteSize))
  {
    Serial.print("\tsector=");
    Serial.print(sector);
    Serial.print("\tsize=");
    Serial.println(byteSize);
/*    if (byteSize >= 512)
    {
      if (RES_OK == mmc::readSectors(sector_buffer, sector, 1))
      {
        for (int i = 0; i < 512; ++i)
        {
          sector_buffer[i] = sector_buffer[i] + 1;
        }
        if (RES_OK == mmc::writeSectors(sector_buffer, sector+1, 1))
        {
          pprint(PSTR("Written to data.bin OK!"));
        }
        else
        {
          pprint(PSTR("Failed to write updated data."));
        }
      }
      else
      {
        pprint(PSTR("Failed to read data.bin."));
      }
    }
    else
    {
      error(PSTR("Found data.bin, but it's too small."));
    }
*/  }
  else
  {
    pprint(PSTR("file not found: "));
    Serial.println(filename);
  }
}

bool showDirectory_walkerfn(directory_entry_t* directory_entry_data, unsigned index, void* user_data)
{
  int* count = (int*)user_data;

  Serial.print(index, DEC);
  Serial.print(' ');

  // Terminate the filename string.  
  // This is deliberately corrupting the buffer data, but that's ok.
  directory_entry_data->filespec[11] = 0;

  unsigned long sector;
  unsigned long byteSize;

  microfat2::getFileInformation(directory_entry_data, sector, byteSize);

  Serial.print(directory_entry_data->filespec);
  Serial.print(" ");
  Serial.print(sector);
  Serial.print(" ");
  Serial.println(byteSize);
  //file_details(directory_entry_data->filespec);
  // Increase 'seen file' count
  *count = (*count)+1;

  // don't stop
  return false;
}

void showDirectory(void)
{
  int count = 0;
  
  pprint(PSTR("Directory of files on card:\n\n"));

  microfat2::walkDirectory(showDirectory_walkerfn, &count);

  pprint(PSTR("\n"));
  Serial.print(count, DEC);
  pprint(PSTR(" files found.\n\n"));
}

void setup(void)
{
  Serial.begin(115200);

  pprint(PSTR("uFat2 demonstration\n"));
  pprint(PSTR("http://arduinonut.blogspot.com\n\n"));

  if (mmc::initialize() != RES_OK)
  {
    error(PSTR("mmc init failed.\n"));
  }

  // Pass in the sector-sized buffer we'll be using ourselves later.
  // uFat doesn't own it, it just needs to use it temporarily.
  // We also pass in the address of a function that is used to read
  // sectors from our device.
  //  
  if (!microfat2::initialize(sector_buffer, &mmc::readSectors))
  {
    error(PSTR("uFat init failed.\n"));
  }
  
  showDirectory();

  file_details(PSTR("DATA    BIN"));
}

void loop(void)
{
  digitalWrite(13, (millis() / 1000) & 1);
}
