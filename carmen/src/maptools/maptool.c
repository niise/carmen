/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, and Sebastian Thrun
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#include <carmen/carmen.h>
#include "map_io.h"

static void usage(char* fmt, ...)
{
  va_list args;

  va_start(args, fmt);
  vfprintf(stderr, fmt, args);
  va_end(args);

  carmen_die("Usage: rotate <rotation in degs> <input file> "
	     "<output file>\n");

  fprintf(stderr, "Error: wrong number of arguments.\n");
  fprintf(stderr, "Usage: maptool mapfilename placename [place params]\n");
  fprintf(stderr, "       2, 3, or 6 place parameters can be given.\n");
  fprintf(stderr, "       2 parameters - named position (x, y)\n");
  fprintf(stderr, "       3 parameters - named pose (x, y, theta)\n");
  fprintf(stderr, "       6 parameters - initialization position\n");
  fprintf(stderr, "                      (x, y, theta, std_x, std_y, "
	  "std_theta)\n");
  fprintf(stderr, "\nRemember: The x/y/std_x/std_y co-ordinates are in "
	  "%sMETRES%s, not in\n", carmen_red_code, carmen_normal_code);
  fprintf(stderr, "grid cells. \n");
  fprintf(stderr, "\nAlso: theta is in %sDEGREES%s. I will print out the "
	  "conversion to radians\n", carmen_red_code, carmen_normal_code);
  fprintf(stderr, "automatically for you.\n");
  
  fprintf(stderr, "Error: wrong number of arguments.\n");

  fprintf(stderr, "Usage: maptool mapfilename chunkname chunkname2 ... \n");
  fprintf(stderr, "chunk name can be one of \"laserscans\", "
	  "\"places\", \n");
  fprintf(stderr, "      \"gridmap\", \"offlimits\", \"expected\"\n");
}

static void toppm(int argc, char *argv[]) 
{
  int err;
  carmen_map_t map;
  char key;

  if(argc != 4) 
    usage("Error: wrong number of parameters.\n");    
  if(carmen_map_read_gridmap_chunk(argv[2], &map) < 0) 
    carmen_die("Error: This file did not contain a gridmap.\n");
  carmen_map_read_offlimits_chunk_into_map(argv[1], &map);

  if (carmen_file_exists(argv[3])) {
    fprintf(stderr, "Overwrite %s? ", argv[3]);
    scanf("%c", &key);
    if (toupper(key) != 'Y')
      exit(-1);
  }

  err = carmen_map_write_to_ppm(&map, argv[3]);
  if (err < 0)
    carmen_die_syserror("Could not write ppm");
}

static void rotate(int argc, char *argv[])
{
  char *in_filename;
  carmen_FILE *in_fp, *out_fp;
  int ret_val;
  carmen_map_t map;
  char *out_filename;
  int rotation = 0;
  double remain;
  int degrees_angle;

  if (argc != 5)
    usage("Error: wrong number of parameters.\n");    

  degrees_angle = (int)(atof(argv[2]) / 90);
  remain = fabs(degrees_angle*90 - atof(argv[2]));
  if (carmen_radians_to_degrees(remain) > 2)
    carmen_die("Rotations only supported in increments of 90 degrees.\n");
  else
    rotation = (int)atof(argv[2]) / 90;

  in_filename = argv[3];
  out_filename = argv[4];

  carmen_warn("Rotating by %d degrees\n", rotation*90);

  ret_val = carmen_map_chunk_exists(in_filename, CARMEN_MAP_GRIDMAP_CHUNK);
	
  if (ret_val < 0)    
      carmen_die_syserror("Couldn't check existence of GRIDMAP_CHUNK in %s", 
			  in_filename);

  if (carmen_map_read_gridmap_chunk(in_filename, &map) < 0)
      carmen_die_syserror("Couldn't read GRIDMAP_CHUNK from %s", in_filename);

  carmen_rotate_gridmap(&map, rotation);

  in_fp = carmen_fopen(in_filename, "r");
  if (in_fp == NULL)
    carmen_die_syserror("Couldn't open %s for reading", in_filename);

  out_fp = carmen_fopen(out_filename, "w");
  if (out_fp == NULL)
    carmen_die_syserror("Couldn't open %s for writing", out_filename);

  ret_val = carmen_map_strip(in_fp, out_fp, CARMEN_MAP_GRIDMAP_CHUNK);

  ret_val = carmen_map_write_gridmap_chunk(out_fp, map.map, map.config.x_size, 
					   map.config.y_size, 
					   map.config.resolution);

  carmen_fclose(in_fp);
  carmen_fclose(out_fp);
}

static void minimize(int argc, char *argv[])
{
  char *filename;
  carmen_FILE *in_fp, *out_fp;
  int ret_val;
  carmen_map_t map;
  int x_offset, y_offset;
  char backup_filename[1024];
  char new_filename[1024];
  carmen_offlimits_list_t offlimits_list;

  if (argc != 3)
    usage("Error: wrong number of parameters.\n");    

  filename = argv[2];

  in_fp = carmen_fopen(filename, "r");
  if (in_fp == NULL)
    carmen_die_syserror("Couldn't open %s for reading", filename);

  sprintf(new_filename, "%s.new", filename);
  out_fp = carmen_fopen(new_filename, "w");
  if (out_fp == NULL)
    carmen_die_syserror("Couldn't open %s for writing", new_filename);

  if (carmen_map_vstrip(in_fp, out_fp, 2, CARMEN_MAP_GRIDMAP_CHUNK,
			CARMEN_MAP_OFFLIMITS_CHUNK) < 0) {
    unlink(new_filename);
    carmen_die_syserror("Couldn't strip map to %s", new_filename);
  }

  ret_val = carmen_map_chunk_exists(filename, CARMEN_MAP_GRIDMAP_CHUNK);

  if (ret_val < 0) {
    unlink(new_filename);
    carmen_die_syserror("Couldn't check existence of GRIDMAP_CHUNK in %s",
			filename);
  }

  if (ret_val ==  0) {
    unlink(new_filename);
    carmen_die("There is no gridmap in %s\n", filename);
  }

  if (carmen_map_read_gridmap_chunk(filename, &map) < 0) {
    unlink(new_filename);
    carmen_die_syserror("Couldn't read GRIDMAP_CHUNK from %s", filename);
  }

  carmen_warn("Map size was %d x %d, ",map.config.x_size, map.config.y_size);

  carmen_map_minimize_gridmap(&map, &x_offset, &y_offset);

  carmen_warn("is now %d x %d (offset %d, %d)\n", map.config.x_size, 
	      map.config.y_size, x_offset, y_offset);

  carmen_map_write_gridmap_chunk(out_fp, map.map, map.config.x_size, 
				 map.config.y_size, map.config.resolution);

  carmen_map_read_offlimits_chunk(filename, &(offlimits_list.offlimits), 
				  &(offlimits_list.list_length));

  carmen_map_move_offlimits_chunk(&offlimits_list, x_offset, y_offset);

  carmen_map_write_offlimits_chunk(out_fp, offlimits_list.offlimits,
				   offlimits_list.list_length);

  carmen_fclose(in_fp);
  carmen_fclose(out_fp);

  sprintf(backup_filename, "%s.bak", filename);
  if (rename(filename, backup_filename) < 0) {
    unlink(new_filename);
    carmen_die("Couldn't rename %s to %s : %s\n", filename, backup_filename, 
	       strerror(errno));
  }
  
  if (rename(new_filename, filename) < 0)
    carmen_die("Couldn't rename %s to %s : %s\n", new_filename, filename, 
						 strerror(errno));
}

static void add_place(int argc, char *argv[])
{
  char *filename, cmd[100];
  char tmp_filename[100];
  carmen_FILE *fp_in, *fp_out;
  carmen_map_placelist_t place_list;
  carmen_place_p places;
  int system_err;

  if(argc != 6 && argc != 7 && argc != 10) 
    usage("Error: wrong number of parameters.\n");    

  filename = argv[2];

  if(!carmen_map_file(filename))
    carmen_die("Error: %s does not appear to be a valid carmen map file;\n" 
	       "if it is gzipped, make sure it has a \".gz\" extension\n",
	       filename);

  if(!carmen_map_chunk_exists(filename, CARMEN_MAP_PLACES_CHUNK)) {
    place_list.num_places = 1;
    place_list.places = (carmen_place_p)calloc(1, sizeof(carmen_place_t)); 
    carmen_test_alloc(place_list.places);
  }
  else {
    carmen_map_read_places_chunk(filename, &place_list);
    place_list.num_places++;
    place_list.places = (carmen_place_p)realloc
      (place_list.places, sizeof(carmen_place_t) * place_list.num_places); /* check_alloc checked */
  }
  carmen_test_alloc(place_list.places);

  places = place_list.places;

  if(argc == 6) {
    strcpy(places[place_list.num_places - 1].name, argv[3]);
    places[place_list.num_places - 1].type = CARMEN_NAMED_POSITION_TYPE;
    places[place_list.num_places - 1].x = atof(argv[4]);
    places[place_list.num_places - 1].y = atof(argv[5]);
  }
  else if(argc == 6) {
    strcpy(places[place_list.num_places - 1].name, argv[3]);
    places[place_list.num_places - 1].type = CARMEN_NAMED_POSE_TYPE;
    places[place_list.num_places - 1].x = atof(argv[4]);
    places[place_list.num_places - 1].y = atof(argv[5]);
    places[place_list.num_places - 1].theta = 
      carmen_degrees_to_radians(atof(argv[6]));
    fprintf(stderr, "Set (%s %s %s) to (%f.2m, %f.2m, %f.2 rad)\n", argv[4],
	    argv[5], argv[6], places[place_list.num_places - 1].x, 
	    places[place_list.num_places - 1].y,
	    places[place_list.num_places - 1].theta);
  }
  else {
    strcpy(places[place_list.num_places - 1].name, argv[3]);
    places[place_list.num_places - 1].type = CARMEN_LOCALIZATION_INIT_TYPE;
    places[place_list.num_places - 1].x = atof(argv[4]);
    places[place_list.num_places - 1].y = atof(argv[5]);
    places[place_list.num_places - 1].theta = 
      carmen_degrees_to_radians(atof(argv[6]));
    places[place_list.num_places - 1].x_std = atof(argv[7]);
    places[place_list.num_places - 1].y_std = atof(argv[8]);
    places[place_list.num_places - 1].theta_std = 
      carmen_degrees_to_radians(atof(argv[9]));

    fprintf(stderr, "Set (%s %s %s %s %s %s) to:\n(%f.2m +/- %f.2m, %f.2m "
	    "+/- %f.2m, %f.2 +/- %f.2 rad)\n", 
	    argv[4], argv[5], argv[6], argv[7], argv[8], argv[9], 
	    places[place_list.num_places - 1].x, 
	    places[place_list.num_places - 1].x_std, 
	    places[place_list.num_places - 1].y, 
	    places[place_list.num_places - 1].y_std,
	    places[place_list.num_places - 1].theta, 
	    places[place_list.num_places - 1].theta_std);
    
  }
  
  fp_in = carmen_fopen(filename, "r");
  if(fp_in == NULL)
    carmen_die_syserror("Error: file %s could not be opened for reading", 
			filename);
  
  strcpy(tmp_filename, "/tmp/newmapXXXXXX");
  system_err = mkstemp(tmp_filename);
  if (system_err == -1) 
    carmen_die_syserror("I need to create a temporary file in /tmp, but I "
			"can't for the\n following reason\n");
  
  fp_out = carmen_fopen(tmp_filename, "w");
  if(fp_out == NULL)
    carmen_die_syserror("Error: file could not be opened for writing");

  if(carmen_map_strip(fp_in, fp_out, CARMEN_MAP_PLACES_CHUNK) < 0)
    carmen_die("Error: could not strip places from map file.\n");
  if(carmen_map_write_places_chunk(fp_out, place_list.places, 
				   place_list.num_places) < 0)
    carmen_die("Error: could not write places chunk.\n");

  carmen_fclose(fp_in);
  carmen_fclose(fp_out);

  sprintf(cmd, "mv -f %s %s", tmp_filename, filename);
  system_err = system(cmd);
  if (system_err != 0) 
    carmen_die("I created a temporary file contained the map with the new "
	       "place name\nat %s. I tried to copy the new file onto your "
	       "old file, but the copy\nfailed for the following reason: %s\n"
	       "\n"
	       "You will have to copy the new file over yourself.\n", 
	       tmp_filename, strerror(errno));
}

static void strip(int argc, char *argv[])
{
  char *filename, cmd[100];
  carmen_FILE *fp_in, *fp_out;
  int index, n;

  if (argc < 4) 
    usage("Error: wrong number of parameters.\n");    

  filename = argv[2];

  fp_in = carmen_fopen(filename, "r");
  if(fp_in == NULL) 
    carmen_die_syserror("Could not open map file for reading");

  fp_out = carmen_fopen("/tmp/stripped.map", "w");
  if(fp_out == NULL) 
    carmen_die_syserror("Could not open temporary file for writing");
  
  n = 0;
  for (index = 3; index < argc; index++)
    {
      if (carmen_strcasecmp(argv[index], "laserscans") == 0) {
	if (carmen_map_strip(fp_in, fp_out, CARMEN_MAP_LASERSCANS_CHUNK) < 0) 
	  carmen_die_syserror("Error: could not strip file");
      }
      else if (carmen_strcasecmp(argv[index], "gridmap") == 0) {
	if (carmen_map_strip(fp_in, fp_out, CARMEN_MAP_GRIDMAP_CHUNK) < 0) 
	  carmen_die_syserror("Error: could not strip file");
      }
      else if (carmen_strcasecmp(argv[index], "offlimits") == 0) {
	if (carmen_map_strip(fp_in, fp_out, CARMEN_MAP_OFFLIMITS_CHUNK) < 0) 
	  carmen_die_syserror("Error: could not strip file");
      }
      else if (carmen_strcasecmp(argv[index], "expected") == 0) {
	if (carmen_map_strip(fp_in, fp_out, CARMEN_MAP_EXPECTED_CHUNK) < 0) 
	  carmen_die_syserror("Error: could not strip file");
      }
      else if (carmen_strcasecmp(argv[index], "places") == 0) {
	if (carmen_map_strip(fp_in, fp_out, CARMEN_MAP_PLACES_CHUNK) < 0) 
	  carmen_die_syserror("Error: could not strip file");
      }
      else
	continue;
      carmen_fclose(fp_in);
      carmen_fclose(fp_out);
      if ((++n)%2) {
	fp_in = carmen_fopen("/tmp/stripped.map", "r");
	if(fp_in == NULL) 
	  carmen_die_syserror("Could not open map file for reading");
	fp_out = carmen_fopen("/tmp/stripped2.map", "w");
	if(fp_out == NULL) 
	  carmen_die_syserror("Could not open temporary file for writing");
      }
      else {
	fp_in = carmen_fopen("/tmp/stripped2.map", "r");
	if(fp_in == NULL)
	  carmen_die_syserror("Could not open map file for reading");
	fp_out = carmen_fopen("/tmp/stripped.map", "w");
	if(fp_out == NULL) 
	  carmen_die_syserror("Could not open temporary file for writing");
      }
    }
 
  carmen_fclose(fp_in);
  carmen_fclose(fp_out);
  if (n % 2)
    sprintf(cmd, "mv -f /tmp/stripped.map %s", filename);
  else
    sprintf(cmd, "mv -f /tmp/stripped2.map %s", filename);
  system(cmd);
}

static void info(int argc, char *argv[])
{
  time_t creation_time;
  char *filename;
  char username[100], origin[100], description[100];
  int chunk_size, i;
  carmen_map_config_t config;
  carmen_map_placelist_t place_list;
  carmen_offlimits_t *offlimits_list;
  int list_length;

  /* Check for the appropriate command line argument */
  if(argc != 2) 
    usage("Error: wrong number of parameters.\n");

  filename = argv[2];

  /* Make sure the file exists */
  if (!carmen_map_file(filename))
    carmen_die("Error: %s does not appear to be a valid carmen map file;\n"
	       "if it is gzipped, make sure it has a \".gz\" extension\n." 
	       "Do you have an incompatible file?\n"
	       "Current map version: %s\n", filename, CARMEN_MAP_VERSION);
  
  /* Print the CREATOR information if the chunk exists */
  if(carmen_map_read_creator_chunk(filename, &creation_time,
				 username, origin, description) == 0) {
    printf("\nMap CREATOR information:\n");
    printf("------------------------\n");
    printf("Map author    : %s\n", username);
    printf("Creation date : %s", asctime(localtime(&creation_time)));
    printf("Origin        : %s\n", origin);
    printf("Description   : %s", description);
    printf("\n");
  }

  /* Print the list of chunks in the file */
  printf("Map contains the following CHUNK types:\n");
  printf("---------------------------------------\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_CREATOR_CHUNK);
  if(chunk_size > 0)
    printf("CREATOR       : yes (%d bytes)\n", chunk_size);
  else
    printf("CREATOR       : no\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_GRIDMAP_CHUNK);
  if(chunk_size > 0)
    {
      carmen_map_read_gridmap_config(filename, &config);
      printf("GRIDMAP       : yes  size : %4d x %4d, resolution : %.2f m/cell (%d bytes)\n", 
	     config.x_size, config.y_size, config.resolution, chunk_size);
    }
  else
    printf("GRIDMAP       : no\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_OFFLIMITS_CHUNK);
  if(chunk_size > 0) {
    printf("OFFLIMITS     : yes (%d bytes)\n", chunk_size);
    carmen_map_read_offlimits_chunk(filename, &offlimits_list, &list_length);
    for (i = 0; i < list_length; i++) {
      if (offlimits_list[i].type == CARMEN_OFFLIMITS_POINT_ID)
	carmen_warn("\tOfflimits %d: point: %d %d\n", i, offlimits_list[i].x1,
		    offlimits_list[i].y1);
      else if (offlimits_list[i].type == CARMEN_OFFLIMITS_LINE_ID)
	carmen_warn("\tOfflimits %d: line: %d %d %d %d\n", i, offlimits_list[i].x1,
		    offlimits_list[i].y1, offlimits_list[i].x2, offlimits_list[i].y2);
      else if (offlimits_list[i].type == CARMEN_OFFLIMITS_RECT_ID)
	carmen_warn("\tOfflimits %d: rect: %d %d %d %d\n", i, offlimits_list[i].x1,
		    offlimits_list[i].y1, offlimits_list[i].x2, offlimits_list[i].y2);
    }
  }
  else
    printf("OFFLIMITS     : no\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_PLACES_CHUNK);
  if(chunk_size > 0)
    printf("PLACES        : yes (%d bytes)\n", chunk_size);
  else
    printf("PLACES        : no\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_EXPECTED_CHUNK);
  if(chunk_size > 0)
    printf("EXPECTED      : yes (%d bytes)\n", chunk_size);
  else
    printf("EXPECTED      : no\n");
  chunk_size = carmen_map_chunk_exists(filename, CARMEN_MAP_LASERSCANS_CHUNK);
  if(chunk_size > 0)
    printf("LASERSCANS    : yes (%d bytes)\n", chunk_size);
  else
    printf("LASERSCANS    : no\n");

  if(carmen_map_chunk_exists(filename, CARMEN_MAP_PLACES_CHUNK)) {
    printf("\nMap Contains the following places:\n");
    printf("----------------------------------\n");
    carmen_map_read_places_chunk(filename, &place_list);
    for(i = 0; i < place_list.num_places; i++)
      if(place_list.places[i].type == CARMEN_NAMED_POSITION_TYPE)
	printf("%-20s (%.1f m, %.1f m)\n", place_list.places[i].name, 
	       place_list.places[i].x, place_list.places[i].y);
      else if(place_list.places[i].type == CARMEN_NAMED_POSE_TYPE)
	printf("%-20s (%.1f m, %.1f m, %.1f deg)\n", 
	       place_list.places[i].name, place_list.places[i].x, 
	       place_list.places[i].y, 
	       carmen_radians_to_degrees(place_list.places[i].theta));
      else if(place_list.places[i].type == CARMEN_LOCALIZATION_INIT_TYPE)
	printf("%-20s (%.1f m, %.1f m, %.1f deg) std = (%.2f m, %.2f m, "
	       "%.2f deg)\n", place_list.places[i].name, 
	       place_list.places[i].x, place_list.places[i].y, 
	       carmen_radians_to_degrees(place_list.places[i].theta), 
	       place_list.places[i].x_std, place_list.places[i].y_std,
	       carmen_radians_to_degrees(place_list.places[i].theta_std));
  }  
}

int main(int argc, char **argv)
{
  char *action;

  if (argc < 2) 
    usage("");  
  
  action = argv[1];
  if (strcmp(action, "toppm") == 0)
    toppm(argc, argv);
  else if (strcmp(action, "rotate") == 0)
    rotate(argc, argv);
  else if (strcmp(action, "minimize") == 0)
    minimize(argc, argv);
  else if (strcmp(action, "add_place") == 0)
    add_place(argc, argv);
  else if (strcmp(action, "strip") == 0)
    strip(argc, argv);
  else if (strcmp(action, "info") == 0)
    info(argc, argv);
  return 0;
}