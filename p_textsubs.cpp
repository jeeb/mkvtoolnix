/*
  mkvmerge -- utility for splicing together matroska files
      from component media subtypes

  r_srt.cpp

  Written by Moritz Bunkus <moritz@bunkus.org>

  Distributed under the GPL
  see the file COPYING for details
  or visit http://www.gnu.org/copyleft/gpl.html
*/

/*!
    \file
    \version \$Id: p_textsubs.cpp,v 1.12 2003/05/04 10:05:41 mosu Exp $
    \brief Subripper subtitle reader
    \author Moritz Bunkus         <moritz @ bunkus.org>
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "common.h"
#include "pr_generic.h"
#include "p_textsubs.h"
#include "matroska.h"

using namespace LIBMATROSKA_NAMESPACE;

textsubs_packetizer_c::textsubs_packetizer_c(track_info_t *nti)
  throw (error_c): generic_packetizer_c(nti) {
  packetno = 0;
}

textsubs_packetizer_c::~textsubs_packetizer_c() {
}

void textsubs_packetizer_c::set_headers() {
  set_serial(-1);
  set_track_type(track_subtitle);
  if (ti->no_utf8_subs)
    set_codec_id(MKV_S_TEXTASCII);
  else
    set_codec_id(MKV_S_TEXTUTF8);

  if (ti->default_track)
    set_as_default_track('s');

  generic_packetizer_c::set_headers();

  track_entry->EnableLacing(false);
}

int textsubs_packetizer_c::process(unsigned char *_subs, int, int64_t start,
                                   int64_t length, int64_t, int64_t) {
  int num_newlines;
  char *subs, *idx1, *idx2;
  int64_t end;

  end = start + length;
  // Adjust the start and end values according to the audio adjustment.
  start += ti->async.displacement;
  start = (int64_t)(ti->async.linear * start);
  end += ti->async.displacement;
  end = (int64_t)(ti->async.linear * end);

  if (end < 0)
    return EMOREDATA;
  else if (start < 0)
    start = 0;

  if (length < 0) {
    fprintf(stderr, "Warning: textsubs_packetizer: Ignoring an entry which "
            "starts after it ends.\n");
    return EMOREDATA;
  }

  // Count the number of lines.
  idx1 = (char *)_subs;
  subs = NULL;
  num_newlines = 0;
  while (*idx1 != 0) {
    if (*idx1 == '\n')
      num_newlines++;
    idx1++;
  }
  subs = (char *)malloc(strlen((char *)_subs) + num_newlines * 2 + 1);
  if (subs == NULL)
    die("malloc");

  // Unify the new lines into DOS style newlines.
  idx1 = (char *)_subs;
  idx2 = subs;
  while (*idx1 != 0) {
    if (*idx1 == '\n') {
      *idx2 = '\r';
      idx2++;
      *idx2 = '\n';
      idx2++;
    } else if (*idx1 != '\r') {
      *idx2 = *idx1;
      idx2++;
    }
    idx1++;
  }
  if (idx2 != subs) {
    while (((idx2 - 1) != subs) &&
           ((*(idx2 - 1) == '\n') || (*(idx2 - 1) == '\r'))) {
      *idx2 = 0;
      idx2--;
    }
  }
  *idx2 = 0;

  if (!ti->no_utf8_subs) {
    char *utf8_subs = to_utf8(subs);
    add_packet((unsigned char *)utf8_subs, strlen(utf8_subs), start, -1, -1,
               length);
    free(utf8_subs);
  } else
    add_packet((unsigned char *)subs, strlen(subs), start, -1, -1, length);

  free(subs);

  return EMOREDATA;
}
