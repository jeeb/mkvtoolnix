/** MPEG video helper functions (MPEG 1, 2 and 4)

   mkvmerge -- utility for splicing together matroska files
   from component media subtypes

   Distributed under the GPL
   see the file COPYING for details
   or visit http://www.gnu.org/copyleft/gpl.html

   Part of this code (the functions \c decode_rbsp_trailing() ) were
   taken or inspired and modified from the ffmpeg project (
   http://ffmpeg.sourceforge.net/index.php ). These functions were
   licensed under the LGPL.

   \file

*/

#include "common/common_pch.h"

#include <unordered_map>

#include "common/bit_cursor.h"
#include "common/byte_buffer.h"
#include "common/checksums.h"
#include "common/endian.h"
#include "common/hacks.h"
#include "common/math.h"
#include "common/mm_io.h"
#include "common/hevc.h"
#include "common/strings/formatting.h"

namespace hevc {

hevcc_c::hevcc_c()
  : m_profile_idc{}
  , m_profile_compat{}
  , m_level_idc{}
  , m_nalu_size_length{}
{
}

hevcc_c::hevcc_c(unsigned int nalu_size_length,
               std::vector<memory_cptr> const &sps_list,
               std::vector<memory_cptr> const &pps_list)
  : m_profile_idc{}
  , m_profile_compat{}
  , m_level_idc{}
  , m_nalu_size_length{nalu_size_length}
  , m_sps_list{sps_list}
  , m_pps_list{pps_list}
{
}

hevcc_c::operator bool()
  const {
  return m_nalu_size_length
      && !m_sps_list.empty()
      && !m_pps_list.empty()
      && (m_sps_info_list.empty() || (m_sps_info_list.size() == m_sps_list.size()))
      && (m_pps_info_list.empty() || (m_pps_info_list.size() == m_pps_list.size()));
}

bool
hevcc_c::parse_vps_list(bool ignore_errors) {
  if (m_vps_info_list.size() == m_vps_list.size())
    return true;

  m_vps_info_list.clear();
  for (auto &vps: m_vps_list) {
    vps_info_t vps_info;
    auto vps_as_rbsp = vps->clone();
    nalu_to_rbsp(vps_as_rbsp);

    if (ignore_errors) {
      try {
        parse_vps(vps_as_rbsp, vps_info);
      } catch (mtx::mm_io::end_of_file_x &) {
      }
    } else if (!parse_vps(vps_as_rbsp, vps_info))
      return false;

    m_vps_info_list.push_back(vps_info);
  }

  return true;
}

bool
hevcc_c::parse_sps_list(bool ignore_errors) {
  if (m_sps_info_list.size() == m_sps_list.size())
    return true;

  m_sps_info_list.clear();
  for (auto &sps: m_sps_list) {
    sps_info_t sps_info;
    auto sps_as_rbsp = sps->clone();
    nalu_to_rbsp(sps_as_rbsp);

    if (ignore_errors) {
      try {
        parse_sps(sps_as_rbsp, sps_info);
      } catch (mtx::mm_io::end_of_file_x &) {
      }
    } else if (!parse_sps(sps_as_rbsp, sps_info))
      return false;

    m_sps_info_list.push_back(sps_info);
  }

  return true;
}

bool
hevcc_c::parse_pps_list(bool ignore_errors) {
  if (m_pps_info_list.size() == m_pps_list.size())
    return true;

  m_pps_info_list.clear();
  for (auto &pps: m_pps_list) {
    pps_info_t pps_info;
    auto pps_as_rbsp = pps->clone();
    nalu_to_rbsp(pps_as_rbsp);

    if (ignore_errors) {
      try {
        parse_pps(pps_as_rbsp, pps_info);
      } catch (mtx::mm_io::end_of_file_x &) {
      }
    } else if (!parse_pps(pps_as_rbsp, pps_info))
      return false;

    m_pps_info_list.push_back(pps_info);
  }

  return true;
}

memory_cptr
hevcc_c::pack() {
  parse_sps_list(true);
  if (!*this)
    return memory_cptr{};

  unsigned int total_size = 6 + 1;

  for (auto &mem : m_sps_list)
    total_size += mem->get_size() + 2;
  for (auto &mem : m_pps_list)
    total_size += mem->get_size() + 2;

  auto destination = memory_c::alloc(total_size);
  auto buffer      = destination->get_buffer();
  auto &sps        = *m_sps_info_list.begin();
  auto write_list  = [&buffer](std::vector<memory_cptr> const &list, uint8_t num_byte_bits) {
    *buffer = list.size() | num_byte_bits;
    ++buffer;

    for (auto &mem : list) {
      auto size = mem->get_size();
      put_uint16_be(buffer, size);
      memcpy(buffer + 2, mem->get_buffer(), size);
      buffer += 2 + size;
    }
  };

  buffer[0] = 1;
  buffer[1] = m_profile_idc    ? m_profile_idc    : sps.profile_idc;
  buffer[2] = m_profile_compat ? m_profile_compat : sps.profile_compat;
  buffer[3] = m_level_idc      ? m_level_idc      : sps.level_idc;
  buffer[4] = 0xfc | (m_nalu_size_length - 1);
  buffer   += 5;

  write_list(m_sps_list, 0xe0);
  write_list(m_pps_list, 0x00);

  return destination;
}

hevcc_c
hevcc_c::unpack(memory_cptr const &mem) {
  hevcc_c hevcc;

  if (!mem || (6 > mem->get_size()))
    return hevcc;

  try {
    mm_mem_io_c in{*mem};

    auto read_list = [&in](std::vector<memory_cptr> &list, unsigned int num_entries_mask) {
      auto num_entries = in.read_uint8() & num_entries_mask;
      while (num_entries) {
        auto size = in.read_uint16_be();
        list.push_back(in.read(size));
        --num_entries;
      }
    };

    in.skip(1);                 // always 1
    hevcc.m_profile_idc      = in.read_uint8();
    hevcc.m_profile_compat   = in.read_uint8();
    hevcc.m_level_idc        = in.read_uint8();
    hevcc.m_nalu_size_length = (in.read_uint8() & 0x03) + 1;

    read_list(hevcc.m_sps_list, 0x0f);
    read_list(hevcc.m_pps_list, 0xff);

    return hevcc;

  } catch (mtx::mm_io::exception &) {
    return hevcc_c{};
  }
}




static const struct {
  int numerator, denominator;
} s_predefined_pars[HEVC_NUM_PREDEFINED_PARS] = {
  {   0,  0 },
  {   1,  1 },
  {  12, 11 },
  {  10, 11 },
  {  16, 11 },
  {  40, 33 },
  {  24, 11 },
  {  20, 11 },
  {  32, 11 },
  {  80, 33 },
  {  18, 11 },
  {  15, 11 },
  {  64, 33 },
  { 160, 99 },
  {   4,  3 },
  {   3,  2 },
  {   2,  1 },
};
};

static int
gecopy(bit_cursor_c &r,
       bit_writer_c &w) {
  int	n = 0, bit;

  while ((bit = r.get_bit()) == 0) {
    w.put_bit(0);
    ++n;
  }

  w.put_bit(1);

  bit = w.copy_bits(n, r);

  return (1 << n) - 1 + bit;
}

static int
geread(bit_cursor_c &r) {
  int	n = 0, bit;

  while ((bit = r.get_bit()) == 0)
    ++n;

  bit = r.get_bits(n);

  return (1 << n) - 1 + bit;
}

static int
sgeread(bit_cursor_c &r) {
  int v = geread(r);
  return v & 1 ? (v + 1) / 2 : -(v / 2);
}

static int
sgecopy(bit_cursor_c &r,
        bit_writer_c &w) {
  int v = gecopy(r, w);
  return v & 1 ? (v + 1) / 2 : -(v / 2);
}

static void
hrdcopy(bit_cursor_c &r,
        bit_writer_c &w) {
  int ncpb = gecopy(r,w);       // cpb_cnt_minus1
  w.copy_bits(4, r);            // bit_rate_scale
  w.copy_bits(4, r);            // cpb_size_scale
  for (int i = 0; i <= ncpb; ++i) {
    gecopy(r,w);                // bit_rate_value_minus1
    gecopy(r,w);                // cpb_size_value_minus1
    w.copy_bits(1, r);          // cbr_flag
  }
  w.copy_bits(5, r);            // initial_cpb_removal_delay_length_minus1
  w.copy_bits(5, r);            // cpb_removal_delay_length_minus1
  w.copy_bits(5, r);            // dpb_output_delay_length_minus1
  w.copy_bits(5, r);            // time_offset_length
}

static void
profile_tier_copy(bit_cursor_c &r,
        bit_writer_c &w,
        unsigned int maxNumSubLayersMinus1) {
  unsigned int i;
  std::vector<bool> sub_layer_profile_present_flag, sub_layer_level_present_flag;

  w.copy_bits(2+1+5, r);  // general_profile_space, general_tier_flag, general_profile_idc
  w.copy_bits(32, r);     // general_profile_compatibility_flag[]
  w.copy_bits(4, r);      // general_progressive_source_flag, general_interlaced_source_flag, general_non_packed_constraint_flag, general_frame_only_constraint_flag
  w.copy_bits(44, r);     // general_reserved_zero_44bits
  w.copy_bits(8, r);      // general_level_idc

  for (i = 0; i < maxNumSubLayersMinus1; i++) {
    sub_layer_profile_present_flag.push_back(w.copy_bits(1, r)); // sub_layer_profile_present_flag[i]
    sub_layer_level_present_flag.push_back(w.copy_bits(1, r));   // sub_layer_level_present_flag[i]
  }

  if (maxNumSubLayersMinus1 > 0)
    for (i = maxNumSubLayersMinus1; i < 8; i++)
      w.copy_bits(2, r);  // reserved_zero_2bits

  for (i = 0; i < maxNumSubLayersMinus1; i++) {
    if (sub_layer_profile_present_flag[i]) {
      w.copy_bits(2+1+5, r);  // sub_layer_profile_space[i], sub_layer_tier_flag[i], sub_layer_profile_idc[i]
      w.copy_bits(32, r);     // sub_layer_profile_compatibility_flag[i][]
      w.copy_bits(4, r);      // sub_layer_progressive_source_flag[i], sub_layer_interlaced_source_flag[i], sub_layer_non_packed_constraint_flag[i], sub_layer_frame_only_constraint_flag[i]
      w.copy_bits(44, r);     // sub_layer_reserved_zero_44bits[i]
    }
    if (sub_layer_level_present_flag[i]) {
      w.copy_bits(8, r);      // sub_layer_level_idc[i]
    }
  }
}

static void
sub_layer_hrd_parameters_copy(bit_cursor_c &r,
        bit_writer_c &w,
        unsigned int CpbCnt,
        bool sub_pic_cpb_params_present_flag) {
  unsigned int i;

  for (i = 0; i <= CpbCnt; i++) {
    gecopy(r, w); // bit_rate_value_minus1[i]
		gecopy(r, w); // cpb_size_value_minus1[i]

    if (sub_pic_cpb_params_present_flag) {
      gecopy(r, w); // cpb_size_du_value_minus1[i]
      gecopy(r, w); // bit_rate_du_value_minus1[i]
		}

    w.copy_bits(1, r); // cbr_flag[i]
  }
}

static void
hrd_parameters_copy(bit_cursor_c &r,
        bit_writer_c &w,
        bool commonInfPresentFlag,
        unsigned int maxNumSubLayersMinus1) {
  unsigned int i;

  bool nal_hrd_parameters_present_flag = false;
  bool vcl_hrd_parameters_present_flag = false;
  bool sub_pic_cpb_params_present_flag = false;

  if(commonInfPresentFlag)
  {
    nal_hrd_parameters_present_flag = w.copy_bits(1, r); // nal_hrd_parameters_present_flag
    vcl_hrd_parameters_present_flag = w.copy_bits(1, r); // vcl_hrd_parameters_present_flag

    if (nal_hrd_parameters_present_flag || vcl_hrd_parameters_present_flag) {
      sub_pic_cpb_params_present_flag = w.copy_bits(1, r); // sub_pic_cpb_params_present_flag
      if (sub_pic_cpb_params_present_flag) {
				  w.copy_bits(8, r);  // tick_divisor_minus2
				  w.copy_bits(5, r);  // du_cpb_removal_delay_increment_length_minus1
				  w.copy_bits(1, r);  // sub_pic_cpb_params_in_pic_timing_sei_flag
				  w.copy_bits(5, r);  // dpb_output_delay_du_length_minus1
      }

      w.copy_bits(4+4, r);  // bit_rate_scale, cpb_size_scale

      if (sub_pic_cpb_params_present_flag)
        w.copy_bits(4, r);  // cpb_size_du_scale

      w.copy_bits(5, r);  // initial_cpb_removal_delay_length_minus1
      w.copy_bits(5, r);  // au_cpb_removal_delay_length_minus1
      w.copy_bits(5, r);  // dpb_output_delay_length_minus1
    }
  }

  for (i = 0; i <= maxNumSubLayersMinus1; i++) {
    bool fixed_pic_rate_general_flag = w.copy_bits(1, r); // fixed_pic_rate_general_flag[i]
    bool fixed_pic_rate_within_cvs_flag = false;
    bool low_delay_hrd_flag = false;
    unsigned int CpbCnt = 0;

    if (!fixed_pic_rate_general_flag)
      fixed_pic_rate_within_cvs_flag = w.copy_bits(1, r); // fixed_pic_rate_within_cvs_flag[i]

    if (fixed_pic_rate_within_cvs_flag)
        gecopy(r, w);                                     // elemental_duration_in_tc_minus1[i]
      else
        low_delay_hrd_flag = w.copy_bits(1, r);           // low_delay_hrd_flag[i]

    if (!low_delay_hrd_flag)
      CpbCnt = gecopy(r, w);                              // cpb_cnt_minus1[i]

    if (nal_hrd_parameters_present_flag)
      sub_layer_hrd_parameters_copy(r, w, CpbCnt, sub_pic_cpb_params_present_flag);

    if (vcl_hrd_parameters_present_flag)
      sub_layer_hrd_parameters_copy(r, w, CpbCnt, sub_pic_cpb_params_present_flag);
  }
}

static void
slcopy(bit_cursor_c &r,
       bit_writer_c &w,
       int size) {
  int delta, next = 8, j;

  for (j = 0; (j < size) && (0 != next); ++j) {
    delta = sgecopy(r, w);
    next  = (next + delta + 256) % 256;
  }
}

void
hevc::sps_info_t::dump() {
  mxinfo(boost::format("sps_info dump:\n"
                       "  id:                                    %1%\n"
                       "  profile_idc:                           %2%\n"
                       "  profile_compat:                        %3%\n"
                       "  level_idc:                             %4%\n"
                       "  log2_max_frame_num:                    %5%\n"
                       "  pic_order_cnt_type:                    %6%\n"
                       "  log2_max_pic_order_cnt_lsb:            %7%\n"
                       "  offset_for_non_ref_pic:                %8%\n"
                       "  offset_for_top_to_bottom_field:        %9%\n"
                       "  num_ref_frames_in_pic_order_cnt_cycle: %10%\n"
                       "  delta_pic_order_always_zero_flag:      %11%\n"
                       "  frame_mbs_only:                        %12%\n"
                       "  vui_present:                           %13%\n"
                       "  ar_found:                              %14%\n"
                       "  par_num:                               %15%\n"
                       "  par_den:                               %16%\n"
                       "  timing_info_present:                   %17%\n"
                       "  num_units_in_tick:                     %18%\n"
                       "  time_scale:                            %19%\n"
                       "  fixed_frame_rate:                      %20%\n"
                       "  crop_left:                             %21%\n"
                       "  crop_top:                              %22%\n"
                       "  crop_right:                            %23%\n"
                       "  crop_bottom:                           %24%\n"
                       "  width:                                 %25%\n"
                       "  height:                                %26%\n"
                       "  checksum:                              %|27$08x|\n")
         % id
         % profile_idc
         % profile_compat
         % level_idc
         % log2_max_frame_num
         % pic_order_cnt_type
         % log2_max_pic_order_cnt_lsb
         % offset_for_non_ref_pic
         % offset_for_top_to_bottom_field
         % num_ref_frames_in_pic_order_cnt_cycle
         % delta_pic_order_always_zero_flag
         % frame_mbs_only
         % vui_present
         % ar_found
         % par_num
         % par_den
         % timing_info_present
         % num_units_in_tick
         % time_scale
         % fixed_frame_rate
         % crop_left
         % crop_top
         % crop_right
         % crop_bottom
         % width
         % height
         % checksum);
}

bool
hevc::sps_info_t::timing_info_valid()
  const {
  return timing_info_present
      && (0 != num_units_in_tick)
      && (0 != time_scale);
}

int64_t
hevc::sps_info_t::default_duration()
  const {
  return 1000000000ll * num_units_in_tick / time_scale;
}

void
hevc::pps_info_t::dump() {
  mxinfo(boost::format("pps_info dump:\n"
                       "id: %1%\n"
                       "sps_id: %2%\n"
                       "pic_order_present: %3%\n"
                       "checksum: %|4$08x|\n")
         % id
         % sps_id
         % pic_order_present
         % checksum);
}

void
hevc::slice_info_t::dump()
  const {
  mxinfo(boost::format("slice_info dump:\n"
                       "  nalu_type:                  %1%\n"
                       "  nal_ref_idc:                %2%\n"
                       "  type:                       %3%\n"
                       "  pps_id:                     %4%\n"
                       "  frame_num:                  %5%\n"
                       "  field_pic_flag:             %6%\n"
                       "  bottom_field_flag:          %7%\n"
                       "  idr_pic_id:                 %8%\n"
                       "  pic_order_cnt_lsb:          %9%\n"
                       "  delta_pic_order_cnt_bottom: %10%\n"
                       "  delta_pic_order_cnt:        %11%\n"
                       "  first_mb_in_slice:          %12%\n"
                       "  sps:                        %13%\n"
                       "  pps:                        %14%\n")
         % static_cast<unsigned int>(nalu_type)
         % static_cast<unsigned int>(nal_ref_idc)
         % static_cast<unsigned int>(type)
         % static_cast<unsigned int>(pps_id)
         % frame_num
         % field_pic_flag
         % bottom_field_flag
         % idr_pic_id
         % pic_order_cnt_lsb
         % delta_pic_order_cnt_bottom
         % (delta_pic_order_cnt[0] << 8 | delta_pic_order_cnt[1])
         % first_mb_in_slice
         % sps
         % pps);
}

void
hevc::nalu_to_rbsp(memory_cptr &buffer) {
  int pos, size = buffer->get_size();
  mm_mem_io_c d(nullptr, size, 100);
  unsigned char *b = buffer->get_buffer();

  for (pos = 0; pos < size; ++pos) {
    if (   ((pos + 2) < size)
        && (0 == b[pos])
        && (0 == b[pos + 1])
        && (3 == b[pos + 2])) {
      d.write_uint8(0);
      d.write_uint8(0);
      pos += 2;

    } else
      d.write_uint8(b[pos]);
  }

  buffer = memory_cptr(new memory_c(d.get_and_lock_buffer(), d.getFilePointer(), true));
}

void
hevc::rbsp_to_nalu(memory_cptr &buffer) {
  int pos, size = buffer->get_size();
  mm_mem_io_c d(nullptr, size, 100);
  unsigned char *b = buffer->get_buffer();

  for (pos = 0; pos < size; ++pos) {
    if (   ((pos + 2) < size)
        && (0 == b[pos])
        && (0 == b[pos + 1])
        && (3 >= b[pos + 2])) {
      d.write_uint8(0);
      d.write_uint8(0);
      d.write_uint8(3);
      ++pos;

    } else
      d.write_uint8(b[pos]);
  }

  buffer = memory_cptr(new memory_c(d.get_and_lock_buffer(), d.getFilePointer(), true));
}

bool 
hevc::parse_vps(memory_cptr &buffer,
                      vps_info_t &vps) {
  int size              = buffer->get_size();
  unsigned char *newsps = (unsigned char *)safemalloc(size + 100);
  memory_cptr mcptr_newsps(new memory_c(newsps, size, true));
  bit_cursor_c r(buffer->get_buffer(), size);
  bit_writer_c w(newsps, size);
  unsigned int i, j;

  memset(&vps, 0, sizeof(vps));

  w.copy_bits(1, r);            // forbidden_zero_bit
  if (w.copy_bits(6, r) != 32)  // nal_unit_type
      return false;
  w.copy_bits(6, r);            // nuh_reserved_zero_6bits
  w.copy_bits(3, r);            // nuh_temporal_id_plus1

  vps.id = w.copy_bits(4, r);                     // vps_video_parameter_set_id
  w.copy_bits(2+6, r);                            // vps_reserved_three_2bits, vps_reserved_zero_6bits
  bool max_sub_layers_minus1 = w.copy_bits(3, r); // vps_max_sub_layers_minus1
  w.copy_bits(1+16, r);                           // vps_temporal_id_nesting_flag, vps_reserved_0xffff_16bits

  profile_tier_copy(r, w, max_sub_layers_minus1); // profile_tier_level(vps_max_sub_layers_minus1)

  bool vps_sub_layer_ordering_info_present_flag = w.copy_bits(1, r);  // vps_sub_layer_ordering_info_present_flag
  for (i = (vps_sub_layer_ordering_info_present_flag ? 
    0 : max_sub_layers_minus1);
    i <= max_sub_layers_minus1; i++) {
    gecopy(r, w); // vps_max_dec_pic_buffering_minus1[i]
    gecopy(r, w); // vps_max_num_reorder_pics[i]
    gecopy(r, w); // vps_max_latency_increase[i]
  }

  unsigned int vps_max_nuh_reserved_zero_layer_id = w.copy_bits(6, r);  // vps_max_nuh_reserved_zero_layer_id
  bool vps_num_op_sets_minus1 = gecopy(r, w);       // vps_num_op_sets_minus1
  for (i = 1; i <= vps_num_op_sets_minus1; i++) {
    for (j = 0; j <= vps_max_nuh_reserved_zero_layer_id; j++) { // operation_point_set(i)
      w.copy_bits(1, r);  // layer_id_included_flag
    }
  }

  if (w.copy_bits(1, r) == 1) { // vps_timing_info_present_flag
    w.copy_bits(32, r);         // vps_num_units_in_tick
    w.copy_bits(32, r);         // vps_time_scale
    if(w.copy_bits(1, r) == 1)  // vps_poc_proportional_to_timing_flag
      gecopy(r, w);             // vps_num_ticks_poc_diff_one_minus1
    unsigned int vps_num_hrd_parameters = gecopy(r, w); // vps_num_hrd_parameters
    for (i = 0; i < vps_num_hrd_parameters; i++) {
      bool cprms_present_flag = false;
      gecopy(r, w);             // hrd_op_set_idx[i]
      if (i > 0)
        cprms_present_flag = w.copy_bits(1, r); // cprms_present_flag[i]
      hrd_parameters_copy(r, w, cprms_present_flag, max_sub_layers_minus1);
    }
  }

  if(w.copy_bits(1, r) == 1)    // vps_extension_flag
    while (!r.eof())
      w.copy_bits(1, r);        // vps_extension_data_flag

  w.put_bit(1);
  w.byte_align();

  buffer = mcptr_newsps;
  buffer->set_size(w.get_bit_position() / 8);

  vps.checksum = calc_adler32(buffer->get_buffer(), buffer->get_size());

  return true;
}

bool
hevc::parse_sps(memory_cptr &buffer,
                      sps_info_t &sps,
                      bool keep_ar_info) {
  std::unordered_map<unsigned int, bool> s_high_level_profile_ids{
    {  44, true }, {  83, true }, {  86, true }, { 100, true }, { 110, true }, { 118, true }, { 122, true }, { 128, true }, { 244, true }
  };

  int size              = buffer->get_size();
  unsigned char *newsps = (unsigned char *)safemalloc(size + 100);
  memory_cptr mcptr_newsps(new memory_c(newsps, size, true));
  bit_cursor_c r(buffer->get_buffer(), size);
  bit_writer_c w(newsps, size);
  int i, nref, mb_width, mb_height;

  keep_ar_info = !hack_engaged(ENGAGE_REMOVE_BITSTREAM_AR_INFO);

  memset(&sps, 0, sizeof(sps));

  sps.par_num  = 1;
  sps.par_den  = 1;
  sps.ar_found = false;

  w.copy_bits(3, r);            // forbidden_zero_bit, nal_ref_idc
  if (w.copy_bits(5, r) != 7)   // nal_unit_type
    return false;
  sps.profile_idc    = w.copy_bits(8, r); // profile_idc
  sps.profile_compat = w.copy_bits(8, r); // constraints
  sps.level_idc      = w.copy_bits(8, r); // level_idc
  sps.id             = gecopy(r, w);      // sps id
  if (s_high_level_profile_ids[sps.profile_idc]) {   // high profile
    if ((sps.chroma_format_idc = gecopy(r, w)) == 3) // chroma_format_idc
      w.copy_bits(1, r);                  // separate_colour_plane_flag
    gecopy(r, w);                         // bit_depth_luma_minus8
    gecopy(r, w);                         // bit_depth_chroma_minus8
    w.copy_bits(1, r);                    // qpprime_y_zero_transform_bypass_flag
    if (w.copy_bits(1, r) == 1)           // seq_scaling_matrix_present_flag
      for (i = 0; i < 8; ++i)
        if (w.copy_bits(1, r) == 1)       // seq_scaling_list_present_flag
          slcopy(r, w, i < 6 ? 16 : 64);
  } else
    sps.chroma_format_idc = 1;            // 4:2:0 assumed

  sps.log2_max_frame_num = gecopy(r, w) + 4; // log2_max_frame_num_minus4
  sps.pic_order_cnt_type = gecopy(r, w);     // pic_order_cnt_type
  switch (sps.pic_order_cnt_type) {
    case 0:
      sps.log2_max_pic_order_cnt_lsb = gecopy(r, w) + 4; // log2_max_pic_order_cnt_lsb_minus4
      break;

    case 1:
      sps.delta_pic_order_always_zero_flag      = w.copy_bits(1, r); // delta_pic_order_always_zero_flag
      sps.offset_for_non_ref_pic                = sgecopy(r, w);     // offset_for_non_ref_pic
      sps.offset_for_top_to_bottom_field        = sgecopy(r, w);     // offset_for_top_to_bottom_field
      nref                                      = gecopy(r, w);      // num_ref_frames_in_pic_order_cnt_cycle
      sps.num_ref_frames_in_pic_order_cnt_cycle = nref;
      for (i = 0; i < nref; ++i)
        gecopy(r, w);           // offset_for_ref_frame
      break;

    case 2:
      break;

    default:
      return false;
  }

  gecopy(r, w);                 // num_ref_frames
  w.copy_bits(1, r);            // gaps_in_frame_num_value_allowed_flag
  mb_width  = gecopy(r, w) + 1; // pic_width_in_mbs_minus1
  mb_height = gecopy(r, w) + 1; // pic_height_in_map_units_minus1
  if (w.copy_bits(1, r) == 0) { // frame_mbs_only_flag
    sps.frame_mbs_only = false;
    w.copy_bits(1, r);          // mb_adaptive_frame_field_flag
  } else
    sps.frame_mbs_only = true;

  sps.width  = mb_width * 16;
  sps.height = (2 - sps.frame_mbs_only) * mb_height * 16;

  w.copy_bits(1, r);            // direct_8x8_inference_flag
  if (w.copy_bits(1, r) == 1) { // frame_cropping_flag
    unsigned int crop_unit_x;
    unsigned int crop_unit_y;
    if (0 == sps.chroma_format_idc) { // monochrome
      crop_unit_x = 1;
      crop_unit_y = 2 - sps.frame_mbs_only;
    } else if (1 == sps.chroma_format_idc) { // 4:2:0
      crop_unit_x = 2;
      crop_unit_y = 2 * (2 - sps.frame_mbs_only);
    } else if (2 == sps.chroma_format_idc) { // 4:2:2
      crop_unit_x = 2;
      crop_unit_y = 2 - sps.frame_mbs_only;
    } else { // 3 == sps.chroma_format_idc   // 4:4:4
      crop_unit_x = 1;
      crop_unit_y = 2 - sps.frame_mbs_only;
    }
    sps.crop_left   = gecopy(r, w); // frame_crop_left_offset
    sps.crop_right  = gecopy(r, w); // frame_crop_right_offset
    sps.crop_top    = gecopy(r, w); // frame_crop_top_offset
    sps.crop_bottom = gecopy(r, w); // frame_crop_bottom_offset

    sps.width      -= crop_unit_x * (sps.crop_left + sps.crop_right);
    sps.height     -= crop_unit_y * (sps.crop_top  + sps.crop_bottom);
  }
  sps.vui_present = w.copy_bits(1, r);
  if (sps.vui_present == 1) {
    if (r.get_bit() == 1) {     // ar_info_present
      int ar_type = r.get_bits(8);

      if (keep_ar_info) {
        w.put_bit(1);
        w.put_bits(8, ar_type);
      } else
        w.put_bit(0);

      sps.ar_found = true;

      if (HEVC_EXTENDED_SAR == ar_type) {
        sps.par_num = r.get_bits(16);
        sps.par_den = r.get_bits(16);

        if (keep_ar_info) {
          w.put_bits(16, sps.par_num);
          w.put_bits(16, sps.par_den);
        }

      } else if (HEVC_NUM_PREDEFINED_PARS >= ar_type) {
        sps.par_num = s_predefined_pars[ar_type].numerator;
        sps.par_den = s_predefined_pars[ar_type].denominator;

      } else
        sps.ar_found = false;

    } else
      w.put_bit(0);             // ar_info_present

    // copy the rest
    if (w.copy_bits(1, r) == 1)   // overscan_info_present
      w.copy_bits(1, r);          // overscan_appropriate
    if (w.copy_bits(1, r) == 1) { // video_signal_type_present
      w.copy_bits(4, r);          // video_format, video_full_range
      if (w.copy_bits(1, r) == 1) // color_desc_present
        w.copy_bits(24, r);
    }
    if (w.copy_bits(1, r) == 1) { // chroma_loc_info_present
      gecopy(r, w);               // chroma_sample_loc_type_top_field
      gecopy(r, w);               // chroma_sample_loc_type_bottom_field
    }
    sps.timing_info_present = w.copy_bits(1, r);
    if (sps.timing_info_present) {
      sps.num_units_in_tick = w.copy_bits(32, r);
      sps.time_scale        = w.copy_bits(32, r);
      sps.fixed_frame_rate  = w.copy_bits(1, r);
    }

    bool f = false;
    if (w.copy_bits(1, r) == 1) { // nal_hrd_parameters_present
      hrdcopy(r, w);
      f = true;
    }
    if (w.copy_bits(1, r) == 1) { // vcl_hrd_parameters_present
      hrdcopy(r, w);
      f = true;
    }
    if (f)
      w.copy_bits(1, r);        // low_delay_hrd_flag
    w.copy_bits(1, r);          // pic_struct_present

    if (w.copy_bits(1, r) == 1) { // bitstream_restriction
      w.copy_bits(1, r);          // motion_vectors_over_pic_boundaries_flag
      gecopy(r, w);               // max_bytes_per_pic_denom
      gecopy(r, w);               // max_bits_per_pic_denom
      gecopy(r, w);               // log2_max_mv_length_h
      gecopy(r, w);               // log2_max_mv_length_v
      gecopy(r, w);               // num_reorder_frames
      gecopy(r, w);               // max_dec_frame_buffering
    }
  }
  
  /*WIP:HEVC pretty much all of this needs migrated to replace stuff in parse_sps
  vps.sar_width = 1;
  vps.sar_height = 1;
  vps.ar_found = false;

  w.copy_bits(1, r);            // forbidden_zero_bit
  if (w.copy_bits(6, r) != 32)  // nal_unit_type
      return false;
  w.copy_bits(6, r);            // nuh_reserved_zero_6bits
  w.copy_bits(3, r);            // nuh_temporal_id_plus1

  if (r.get_bit() == 1) {                   // aspect_ratio_info_present_flag
    unsigned int ar_type = r.get_bits(8);   // aspect_ratio_idc

    if (keep_ar_info) {
      w.put_bit(1);
      w.put_bits(8, ar_type);
    } else
      w.put_bit(0);

    vps.ar_found = true;

    if (HEVC_EXTENDED_SAR == ar_type) {
      vps.sar_width = r.get_bits(16);   // sar_width
      vps.sar_height = r.get_bits(16);  // sar_height

      if (keep_ar_info) {
        w.put_bits(16, vps.sar_width);
        w.put_bits(16, vps.sar_height);
      }
    } else if (HEVC_NUM_PREDEFINED_PARS >= ar_type) {
      vps.sar_width = s_predefined_pars[ar_type].numerator;
      vps.sar_height = s_predefined_pars[ar_type].denominator;
    } else
      vps.ar_found = false;
  }

  // copy the rest
  if (w.copy_bits(1, r) == 1)   // overscan_info_present_flag
    w.copy_bits(1, r);          // overscan_appropriate_flag
  if (w.copy_bits(1, r) == 1) { // video_signal_type_present_flag
    w.copy_bits(4, r);          // video_format, video_full_range_flag
    if (w.copy_bits(1, r) == 1) // color_desc_present_flag
      w.copy_bits(24, r);       // colour_primaries, transfer_characteristics, matrix_coefficients
  }
  if (w.copy_bits(1, r) == 1) { // chroma_loc_info_present_flag
    gecopy(r, w);               // chroma_sample_loc_type_top_field
    gecopy(r, w);               // chroma_sample_loc_type_bottom_field
  }
  w.copy_bits(3, r);            // neutral_chroma_indication_flag, field_seq_flag, frame_field_info_present_flag
  if (w.copy_bits(1, r) == 1) { // default_display_window_flag
    gecopy(r, w);               // def_disp_win_left_offset
    gecopy(r, w);               // def_disp_win_right_offset
    gecopy(r, w);               // def_disp_win_top_offset
    gecopy(r, w);               // def_disp_win_bottom_offset
  }
  vps.vui_timing_info_present_flag = w.copy_bits(1, r); // vui_timing_info_present_flag
  if (vps.vui_timing_info_present_flag) {
    vps.vui_num_units_in_tick = w.copy_bits(32, r);     // vui_num_units_in_tick
    vps.vui_time_scale        = w.copy_bits(32, r);     // vui_time_scale
    if (w.copy_bits(1, r) == 1) { // vui_poc_proportional_to_timing_flag
      gecopy(r, w); // vui_num_ticks_poc_diff_one_minus1
    }
    if (w.copy_bits(1, r) == 1) { // vui_hrd_parameters_present_flag
      hrdcopy(r, w);//WIP:HEVC hrdcopy needs new implementation
    }
    if (w.copy_bits(1, r) == 1) { // bitstream_restriction_flag
      w.copy_bits(3, r);  // tiles_fixed_structure_flag, motion_vectors_over_pic_boundaries_flag, restricted_ref_pic_lists_flag
      gecopy(r, w); // min_spatial_segmentation_idc
      gecopy(r, w); // max_bytes_per_pic_denom
      gecopy(r, w); // max_bits_per_mincu_denom
      gecopy(r, w); // log2_max_mv_length_horizontal
      gecopy(r, w); // log2_max_mv_length_vertical
    }
  }
  */
  w.put_bit(1);
  w.byte_align();

  buffer = mcptr_newsps;
  buffer->set_size(w.get_bit_position() / 8);

  sps.checksum = calc_adler32(buffer->get_buffer(), buffer->get_size());

  return true;
}

bool
hevc::parse_pps(memory_cptr &buffer,
                      pps_info_t &pps) {
  try {
    bit_cursor_c r(buffer->get_buffer(), buffer->get_size());

    memset(&pps, 0, sizeof(pps));

    r.skip_bits(3);             // forbidden_zero_bit, nal_ref_idc
    if (r.get_bits(5) != 8)     // nal_unit_type
      return false;
    pps.id     = geread(r);
    pps.sps_id = geread(r);

    r.skip_bits(1);             // entropy_coding_mode_flag
    pps.pic_order_present = r.get_bit();
    pps.checksum          = calc_adler32(buffer->get_buffer(), buffer->get_size());

    return true;
  } catch (...) {
    return false;
  }
}

/** Extract the pixel aspect ratio from the HEVC codec data

   This function searches a buffer containing the HEVC codec
   initialization for the pixel aspectc ratio. If it is found then the
   numerator and the denominator are returned, and the aspect ratio
   information is removed from the buffer. The new buffer is returned
   in the variable \c buffer, and the new size is returned in \c buffer_size.

   \param buffer The buffer containing the HEVC codec data.
   \param buffer_size The size of the buffer in bytes.
   \param par_num The numerator, if found, is stored in this variable.
   \param par_den The denominator, if found, is stored in this variable.

   \return \c true if the pixel aspect ratio was found and \c false
     otherwise.
*/
bool
hevc::extract_par(uint8_t *&buffer,
                        size_t &buffer_size,
                        uint32_t &par_num,
                        uint32_t &par_den) {
  try {
    auto hevcc     = hevcc_c::unpack(memory_cptr{new memory_c{buffer, buffer_size, false}});
    auto new_hevcc = hevcc;
    par_num       = 1;
    par_den       = 1;
    bool ar_found = false;

    new_hevcc.m_sps_list.clear();

    for (auto &nalu : hevcc.m_sps_list) {
      if (!ar_found) {
        nalu_to_rbsp(nalu);

        try {
          sps_info_t sps_info;
          if (hevc::parse_sps(nalu, sps_info)) {
            ar_found = sps_info.ar_found;
            if (ar_found) {
              par_num = sps_info.par_num;
              par_den = sps_info.par_den;
            }
          }
        } catch (mtx::mm_io::end_of_file_x &) {
        }

        rbsp_to_nalu(nalu);
      }

      new_hevcc.m_sps_list.push_back(nalu);
    }

    auto packed_new_hevcc = new_hevcc.pack();
    if (packed_new_hevcc) {
      buffer_size = packed_new_hevcc->get_size();
      buffer      = packed_new_hevcc->get_buffer();
      packed_new_hevcc->lock();
    }

    return ar_found;

  } catch(...) {
    return false;
  }
}

bool
hevc::is_hevc_fourcc(const char *fourcc) {
  return !strncasecmp(fourcc, "hevc",  4);
}

memory_cptr
hevc::hevcc_to_nalus(const unsigned char *buffer,
                          size_t size) {
  try {
    if (6 > size)
      throw false;

    uint32_t marker = get_uint32_be(buffer);
    if (((marker & 0xffffff00) == 0x00000100) || (0x00000001 == marker))
      return memory_c::clone(buffer, size);

    mm_mem_io_c mem(buffer, size);
    byte_buffer_c nalus(size * 2);

    if (0x01 != mem.read_uint8())
      throw false;

    mem.setFilePointer(4, seek_beginning);
    size_t nal_size_size = 1 + (mem.read_uint8() & 3);
    if (2 > nal_size_size)
      throw false;

    size_t sps_or_pps;
    for (sps_or_pps = 0; 2 > sps_or_pps; ++sps_or_pps) {
      unsigned int num = mem.read_uint8();
      if (0 == sps_or_pps)
        num &= 0x1f;

      size_t i;
      for (i = 0; num > i; ++i) {
        uint16_t element_size   = mem.read_uint16_be();
        memory_cptr copy_buffer = memory_c::alloc(element_size + 4);
        if (element_size != mem.read(copy_buffer->get_buffer() + 4, element_size))
          throw false;

        put_uint32_be(copy_buffer->get_buffer(), NALU_START_CODE);
        nalus.add(copy_buffer->get_buffer(), element_size + 4);
      }
    }

    if (mem.getFilePointer() == size)
      return memory_c::clone(nalus.get_buffer(), nalus.get_size());

  } catch (...) {
  }

  return memory_cptr{};
}

hevc::hevc_es_parser_c::hevc_es_parser_c()
  : m_nalu_size_length(4)
  , m_keep_ar_info(true)
  , m_hevcc_ready(false)
  , m_hevcc_changed(false)
  , m_stream_default_duration(-1)
  , m_forced_default_duration(-1)
  , m_container_default_duration(-1)
  , m_frame_number(0)
  , m_num_skipped_frames(0)
  , m_first_keyframe_found(false)
  , m_recovery_point_valid(false)
  , m_b_frames_since_keyframe(false)
  , m_par_found(false)
  , m_max_timecode(0)
  , m_stream_position(0)
  , m_parsed_position(0)
  , m_have_incomplete_frame(false)
  , m_ignore_nalu_size_length_errors(false)
  , m_discard_actual_frames(false)
  , m_debug_keyframe_detection(debugging_requested("hevc_parser|hevc_keyframe_detection"))
  , m_debug_nalu_types(        debugging_requested("hevc_parser|hevc_nalu_types"))
  , m_debug_timecodes(         debugging_requested("hevc_parser|hevc_timecodes"))
  , m_debug_sps_info(          debugging_requested("hevc_parser|hevc_sps|hevc_sps_info"))
{
  if (m_debug_nalu_types)
    init_nalu_names();
}

hevc::hevc_es_parser_c::~hevc_es_parser_c() {
  mxdebug_if(debugging_requested("hevc_statistics"),
             boost::format("HEVC statistics: #frames: out %1% discarded %2% #timecodes: in %3% generated %4% discarded %5% num_fields: %6% num_frames: %7%\n")
             % m_stats.num_frames_out % m_stats.num_frames_discarded % m_stats.num_timecodes_in % m_stats.num_timecodes_generated % m_stats.num_timecodes_discarded
             % m_stats.num_field_slices % m_stats.num_frame_slices);

  mxdebug_if(m_debug_timecodes, boost::format("stream_position %1% parsed_position %2%\n") % m_stream_position % m_parsed_position);

  if (!debugging_requested("hevc_num_slices_by_type"))
    return;

  static const char *s_type_names[] = {
    "P",  "B",  "I",  "SP",  "SI",
    "P2", "B2", "I2", "SP2", "SI2",
    "unknown"
  };

  int i;
  mxdebug("hevc: Number of slices by type:\n");
  for (i = 0; 10 >= i; ++i)
    if (0 != m_stats.num_slices_by_type[i])
      mxdebug(boost::format("  %1%: %2%\n") % s_type_names[i] % m_stats.num_slices_by_type[i]);
}

void
hevc::hevc_es_parser_c::discard_actual_frames(bool discard) {
  m_discard_actual_frames = discard;
}

void
hevc::hevc_es_parser_c::add_bytes(unsigned char *buffer,
                                       size_t size) {
  memory_slice_cursor_c cursor;
  int marker_size              = 0;
  int previous_marker_size     = 0;
  int previous_pos             = -1;
  uint64_t previous_parsed_pos = m_parsed_position;

  if (m_unparsed_buffer && (0 != m_unparsed_buffer->get_size()))
    cursor.add_slice(m_unparsed_buffer);
  cursor.add_slice(buffer, size);

  if (3 <= cursor.get_remaining_size()) {
    uint32_t marker =                               1 << 24
                    | (unsigned int)cursor.get_char() << 16
                    | (unsigned int)cursor.get_char() <<  8
                    | (unsigned int)cursor.get_char();

    while (1) {
      if (NALU_START_CODE == marker)
        marker_size = 4;
      else if (NALU_START_CODE == (marker & 0x00ffffff))
        marker_size = 3;

      if (0 != marker_size) {
        if (-1 != previous_pos) {
          int new_size = cursor.get_position() - marker_size - previous_pos - previous_marker_size;
          memory_cptr nalu(new memory_c(safemalloc(new_size), new_size, true));
          cursor.copy(nalu->get_buffer(), previous_pos + previous_marker_size, new_size);
          m_parsed_position = previous_parsed_pos + previous_pos;
          handle_nalu(nalu);
        }
        previous_pos         = cursor.get_position() - marker_size;
        previous_marker_size = marker_size;
        marker_size          = 0;
      }

      if (!cursor.char_available())
        break;

      marker <<= 8;
      marker  |= (unsigned int)cursor.get_char();
    }
  }

  if (-1 == previous_pos)
    previous_pos = 0;

  m_stream_position += size;
  m_parsed_position  = previous_parsed_pos + previous_pos;

  int new_size = cursor.get_size() - previous_pos;
  if (0 != new_size) {
    m_unparsed_buffer = memory_cptr(new memory_c(safemalloc(new_size), new_size, true));
    cursor.copy(m_unparsed_buffer->get_buffer(), previous_pos, new_size);

  } else
    m_unparsed_buffer.reset();
}

void
hevc::hevc_es_parser_c::flush() {
  if (m_unparsed_buffer && (5 <= m_unparsed_buffer->get_size())) {
    m_parsed_position += m_unparsed_buffer->get_size();
    int marker_size = get_uint32_be(m_unparsed_buffer->get_buffer()) == NALU_START_CODE ? 4 : 3;
    handle_nalu(memory_c::clone(m_unparsed_buffer->get_buffer() + marker_size, m_unparsed_buffer->get_size() - marker_size));
  }

  m_unparsed_buffer.reset();
  if (m_have_incomplete_frame) {
    m_frames.push_back(m_incomplete_frame);
    m_have_incomplete_frame = false;
  }

  cleanup();
}

void
hevc::hevc_es_parser_c::add_timecode(int64_t timecode) {
  m_provided_timecodes.push_back(timecode);
  m_provided_stream_positions.push_back(m_stream_position);
  ++m_stats.num_timecodes_in;
}

void
hevc::hevc_es_parser_c::write_nalu_size(unsigned char *buffer,
                                             size_t size,
                                             int this_nalu_size_length)
  const {
  unsigned int nalu_size_length = -1 == this_nalu_size_length ? m_nalu_size_length : this_nalu_size_length;

  if (!m_ignore_nalu_size_length_errors && (size >= ((uint64_t)1 << (nalu_size_length * 8)))) {
    unsigned int required_bytes = nalu_size_length + 1;
    while (size >= (1u << (required_bytes * 8)))
      ++required_bytes;

    throw nalu_size_length_x(required_bytes);
  }

  unsigned int i;
  for (i = 0; i < nalu_size_length; i++)
    buffer[i] = (size >> (8 * (nalu_size_length - 1 - i))) & 0xff;
}

bool
hevc::hevc_es_parser_c::flush_decision(slice_info_t &si,
                                            slice_info_t &ref) {

  if (HEVC_NALU_TYPE_IDR_W_RADL == si.nalu_type) {
    if (0 != si.first_mb_in_slice)
      return false;

    if ((HEVC_NALU_TYPE_IDR_W_RADL == ref.nalu_type) && (si.idr_pic_id != ref.idr_pic_id))
      return true;

    if (HEVC_NALU_TYPE_IDR_W_RADL != ref.nalu_type)
      return true;
  }

  if (si.frame_num != ref.frame_num)
    return true;
  if (si.field_pic_flag != ref.field_pic_flag)
    return true;
  if ((si.nal_ref_idc != ref.nal_ref_idc) && (!si.nal_ref_idc || !ref.nal_ref_idc))
    return true;

  if (m_sps_info_list[si.sps].pic_order_cnt_type == m_sps_info_list[ref.sps].pic_order_cnt_type) {
    if (!m_sps_info_list[ref.sps].pic_order_cnt_type) {
      if (si.pic_order_cnt_lsb != ref.pic_order_cnt_lsb)
        return true;
      if (si.delta_pic_order_cnt_bottom != ref.delta_pic_order_cnt_bottom)
        return true;

    } else if ((1 == m_sps_info_list[ref.sps].pic_order_cnt_type)
               && (   (si.delta_pic_order_cnt[0] != ref.delta_pic_order_cnt[0])
                   || (si.delta_pic_order_cnt[1] != ref.delta_pic_order_cnt[1])))
      return true;
  }

  return false;
}

void
hevc::hevc_es_parser_c::flush_incomplete_frame() {
  if (!m_have_incomplete_frame || !m_hevcc_ready)
    return;

  m_frames.push_back(m_incomplete_frame);
  m_incomplete_frame.clear();
  m_have_incomplete_frame = false;
}

void
hevc::hevc_es_parser_c::flush_unhandled_nalus() {
  std::deque<memory_cptr>::iterator nalu = m_unhandled_nalus.begin();

  while (m_unhandled_nalus.end() != nalu) {
    handle_nalu(*nalu);
    nalu++;
  }

  m_unhandled_nalus.clear();
}

void
hevc::hevc_es_parser_c::handle_slice_nalu(memory_cptr &nalu) {
  if (!m_hevcc_ready) {
    m_unhandled_nalus.push_back(nalu);
    return;
  }

  slice_info_t si;
  if (!parse_slice(nalu, si))
    return;

  if (m_have_incomplete_frame && flush_decision(si, m_incomplete_frame.m_si))
    flush_incomplete_frame();

  if (m_have_incomplete_frame) {
    memory_c &mem = *(m_incomplete_frame.m_data.get());
    int offset    = mem.get_size();
    mem.resize(offset + m_nalu_size_length + nalu->get_size());
    write_nalu_size(mem.get_buffer() + offset, nalu->get_size());
    memcpy(mem.get_buffer() + offset + m_nalu_size_length, nalu->get_buffer(), nalu->get_size());

    return;
  }

  bool is_i_slice =  (HEVC_SLICE_TYPE_I   == si.type)
                  || (HEVC_SLICE_TYPE2_I  == si.type)
                  || (HEVC_SLICE_TYPE_SI  == si.type)
                  || (HEVC_SLICE_TYPE2_SI == si.type);
  bool is_b_slice =  (HEVC_SLICE_TYPE_B   == si.type)
                  || (HEVC_SLICE_TYPE2_B  == si.type);

  m_incomplete_frame.m_si       =  si;
  m_incomplete_frame.m_keyframe =  m_recovery_point_valid
                                || (   is_i_slice
                                    && (   (m_debug_keyframe_detection && !m_b_frames_since_keyframe)
                                        || (HEVC_NALU_TYPE_IDR_W_RADL == si.nalu_type)));
  m_recovery_point_valid        =  false;

  if (m_incomplete_frame.m_keyframe) {
    m_first_keyframe_found    = true;
    m_b_frames_since_keyframe = false;
    cleanup();

  } else
    m_b_frames_since_keyframe |= is_b_slice;

  m_incomplete_frame.m_data = create_nalu_with_size(nalu, true);
  m_have_incomplete_frame   = true;

  if (!m_provided_stream_positions.empty() && (m_parsed_position >= m_provided_stream_positions.front())) {
    m_incomplete_frame.m_has_provided_timecode = true;
    m_provided_stream_positions.pop_front();
  }

  ++m_frame_number;
}
void
hevc::hevc_es_parser_c::handle_vps_nalu(memory_cptr &nalu) {
  vps_info_t vps_info;

  nalu_to_rbsp(nalu);
  if (!parse_vps(nalu, vps_info))
    return;
  rbsp_to_nalu(nalu);

  size_t i;
  for (i = 0; m_vps_info_list.size() > i; ++i)
    if (m_vps_info_list[i].id == vps_info.id)
      break;

  if (m_vps_info_list.size() == i) {
    m_vps_list.push_back(nalu);
    m_vps_info_list.push_back(vps_info);
    m_hevcc_changed = true;

  } else if (m_vps_info_list[i].checksum != vps_info.checksum) {
    mxverb(2, boost::format("hevc: VPS ID %|1$04x| changed; checksum old %|2$04x| new %|3$04x|\n") % vps_info.id % m_vps_info_list[i].checksum % vps_info.checksum);

    m_vps_info_list[i] = vps_info;
    m_vps_list[i]      = nalu;
    m_hevcc_changed     = true;
  }

  m_extra_data.push_back(create_nalu_with_size(nalu));
}

void
hevc::hevc_es_parser_c::handle_sps_nalu(memory_cptr &nalu) {
  sps_info_t sps_info;

//WIP: HEVC
  m_sps_list.push_back(nalu);
  m_sps_info_list.push_back(sps_info);

  mxinfo("Parsing of HEVC SPS not complete\n");
  return;
//WIP: HEVC
  nalu_to_rbsp(nalu);
  if (!parse_sps(nalu, sps_info, m_keep_ar_info))
    return;
  rbsp_to_nalu(nalu);

  size_t i;
  for (i = 0; m_sps_info_list.size() > i; ++i)
    if (m_sps_info_list[i].id == sps_info.id)
      break;

  bool use_sps_info = true;
  if (m_sps_info_list.size() == i) {
    m_sps_list.push_back(nalu);
    m_sps_info_list.push_back(sps_info);
    m_hevcc_changed = true;

  } else if (m_sps_info_list[i].checksum != sps_info.checksum) {
    mxverb(2, boost::format("hevc: SPS ID %|1$04x| changed; checksum old %|2$04x| new %|3$04x|\n") % sps_info.id % m_sps_info_list[i].checksum % sps_info.checksum);

    m_sps_info_list[i] = sps_info;
    m_sps_list[i]      = nalu;
    m_hevcc_changed     = true;

  } else
    use_sps_info = false;

  m_extra_data.push_back(create_nalu_with_size(nalu));

  if (use_sps_info && m_debug_sps_info)
    sps_info.dump();

  if (!use_sps_info)
    return;

  if (   !has_stream_default_duration()
      && sps_info.timing_info_valid()) {
    m_stream_default_duration = sps_info.default_duration();
    mxdebug_if(m_debug_timecodes, boost::format("Stream default duration: %1%\n") % m_stream_default_duration);
  }

  if (   !m_par_found
      && sps_info.ar_found
      && (0 != sps_info.par_den)) {
    m_par_found = true;
    m_par       = int64_rational_c(sps_info.par_num, sps_info.par_den);
  }
}

void
hevc::hevc_es_parser_c::handle_pps_nalu(memory_cptr &nalu) {
  pps_info_t pps_info;

  m_pps_list.push_back(nalu);
  m_pps_info_list.push_back(pps_info);

  mxinfo("Parsing of HEVC PPS not complete\n");
  return;

  nalu_to_rbsp(nalu);
  if (!parse_pps(nalu, pps_info))
    return;
  rbsp_to_nalu(nalu);

  size_t i;
  for (i = 0; m_pps_info_list.size() > i; ++i)
    if (m_pps_info_list[i].id == pps_info.id)
      break;

  if (m_pps_info_list.size() == i) {
    m_pps_list.push_back(nalu);
    m_pps_info_list.push_back(pps_info);
    m_hevcc_changed = true;

  } else if (m_pps_info_list[i].checksum != pps_info.checksum) {
    mxverb(2, boost::format("hevc: PPS ID %|1$04x| changed; checksum old %|2$04x| new %|3$04x|\n") % pps_info.id % m_pps_info_list[i].checksum % pps_info.checksum);

    m_pps_info_list[i] = pps_info;
    m_pps_list[i]      = nalu;
    m_hevcc_changed     = true;
  }

  m_extra_data.push_back(create_nalu_with_size(nalu));
}

void
hevc::hevc_es_parser_c::handle_sei_nalu(memory_cptr &nalu) {
  try {
    nalu_to_rbsp(nalu);

    bit_cursor_c r(nalu->get_buffer(), nalu->get_size());

    r.skip_bits(8);

    while (1) {
      int ptype = 0;
      int value;
      while ((value = r.get_bits(8)) == 0xff)
        ptype += value;
      ptype += value;

      int psize = 0;
      while ((value = r.get_bits(8)) == 0xff)
        psize += value;
      psize += value;

      if (6 == ptype) {         // recovery point
        m_recovery_point_valid = true;
        return;
      } else if (0x80 == ptype)
        return;

      r.skip_bits(psize * 8);
    }
  } catch (...) {
  }
}

void
hevc::hevc_es_parser_c::handle_nalu(memory_cptr nalu) {
  if (1 > nalu->get_size())
    return;

  int type = (*(nalu->get_buffer()) >> 1) & 0x3F;

  mxdebug_if(m_debug_nalu_types, boost::format("NALU type 0x%|1$02x| (%2%) size %3%\n") % type % get_nalu_type_name(type) % nalu->get_size());

  switch (type) {
   case HEVC_NALU_TYPE_VIDEO_PARAM:
      flush_incomplete_frame();
      handle_vps_nalu(nalu);
      break;

   case HEVC_NALU_TYPE_SEQ_PARAM:
      flush_incomplete_frame();
      handle_sps_nalu(nalu);
      break;

    case HEVC_NALU_TYPE_PIC_PARAM:
      flush_incomplete_frame();
      handle_pps_nalu(nalu);
      break;

    case HEVC_NALU_TYPE_END_OF_SEQ:
    case HEVC_NALU_TYPE_END_OF_STREAM:
    case HEVC_NALU_TYPE_ACCESS_UNIT:
      flush_incomplete_frame();
      break;

    case HEVC_NALU_TYPE_FILLER_DATA:
      // Skip these.
      break;

    case HEVC_NALU_TYPE_TRAIL_N:
    case HEVC_NALU_TYPE_TRAIL_R:
//    case HEVC_NALU_TYPE_DP_B_SLICE:
//    case HEVC_NALU_TYPE_DP_C_SLICE:
    case HEVC_NALU_TYPE_IDR_W_RADL:
      if (!m_hevcc_ready && !m_sps_info_list.empty() && !m_pps_info_list.empty() && !m_vps_info_list.empty()) {
        m_hevcc_ready = true;
        flush_unhandled_nalus();
      }
      handle_slice_nalu(nalu);
      break;

    default:
      flush_incomplete_frame();
      if (!m_hevcc_ready && !m_sps_info_list.empty() && !m_pps_info_list.empty() && !m_vps_info_list.empty()) {
        m_hevcc_ready = true;
        flush_unhandled_nalus();
      }
      m_extra_data.push_back(create_nalu_with_size(nalu));

      if (HEVC_NALU_TYPE_PREFIX_SEI == type)
        handle_sei_nalu(nalu);

      break;
  }
}

bool
hevc::hevc_es_parser_c::parse_slice(memory_cptr &buffer,
                                         slice_info_t &si) {
  try {
    bit_cursor_c r(buffer->get_buffer(), buffer->get_size());

    memset(&si, 0, sizeof(si));

    r.get_bits(1); //Forbidden zero bit
    //si.nal_ref_idc = r.get_bits(3); // forbidden_zero_bit, nal_ref_idc
    si.nalu_type   = r.get_bits(6); // si.nalu_type
    if (   (HEVC_NALU_TYPE_TRAIL_N != si.nalu_type)
        && (HEVC_NALU_TYPE_TRAIL_R != si.nalu_type)
        &&  (HEVC_NALU_TYPE_IDR_W_RADL != si.nalu_type))
      return false;

    si.first_mb_in_slice = geread(r); // first_mb_in_slice
    si.type              = geread(r); // slice_type

    ++m_stats.num_slices_by_type[9 < si.type ? 10 : si.type];

    if (9 < si.type) {
      mxverb(3, boost::format("slice parser error: 9 < si.type: %1%\n") % si.type);
      return false;
    }

    si.pps_id = geread(r);      // pps_id

    size_t pps_idx;
    for (pps_idx = 0; m_pps_info_list.size() > pps_idx; ++pps_idx)
      if (m_pps_info_list[pps_idx].id == si.pps_id)
        break;
    if (m_pps_info_list.size() == pps_idx) {
      mxverb(3, boost::format("slice parser error: PPS not found: %1%\n") % si.pps_id);
      return false;
    }

    pps_info_t &pps = m_pps_info_list[pps_idx];
    size_t sps_idx;
    for (sps_idx = 0; m_sps_info_list.size() > sps_idx; ++sps_idx)
      if (m_sps_info_list[sps_idx].id == pps.sps_id)
        break;
    if (m_sps_info_list.size() == sps_idx)
      return false;

    si.sps = sps_idx;
    si.pps = pps_idx;

    sps_info_t &sps = m_sps_info_list[sps_idx];

    si.frame_num = r.get_bits(sps.log2_max_frame_num);

    if (!sps.frame_mbs_only) {
      si.field_pic_flag = r.get_bit();
      if (si.field_pic_flag)
        si.bottom_field_flag = r.get_bit();
    }

    if (HEVC_NALU_TYPE_IDR_W_RADL == si.nalu_type)
      si.idr_pic_id = geread(r);

    if (0 == sps.pic_order_cnt_type) {
      si.pic_order_cnt_lsb = r.get_bits(sps.log2_max_pic_order_cnt_lsb);
      if (pps.pic_order_present && !si.field_pic_flag)
        si.delta_pic_order_cnt_bottom = sgeread(r);
    }

    if ((1 == sps.pic_order_cnt_type) && !sps.delta_pic_order_always_zero_flag) {
      si.delta_pic_order_cnt[0] = sgeread(r);
      if (pps.pic_order_present && !si.field_pic_flag)
        si.delta_pic_order_cnt[1] = sgeread(r);
    }

    return true;
  } catch (...) {
    return false;
  }
}

int64_t
hevc::hevc_es_parser_c::duration_for(slice_info_t const &si)
  const {
  int64_t duration = -1 != m_forced_default_duration                                                  ? m_forced_default_duration
                   : (m_sps_info_list.size() > si.sps) && m_sps_info_list[si.sps].timing_info_valid() ? m_sps_info_list[si.sps].default_duration()
                   : -1 != m_stream_default_duration                                                  ? m_stream_default_duration
                   : -1 != m_container_default_duration                                               ? m_container_default_duration
                   :                                                                                    20000000;
  return duration * (si.field_pic_flag ? 1 : 2);
}

int64_t
hevc::hevc_es_parser_c::get_most_often_used_duration()
  const {
  int64_t const s_common_default_durations[] = {
    1000000000ll / 50,
    1000000000ll / 25,
    1000000000ll / 60,
    1000000000ll / 30,
    1000000000ll * 1001 / 48000,
    1000000000ll * 1001 / 24000,
    1000000000ll * 1001 / 60000,
    1000000000ll * 1001 / 30000,
  };

  auto most_often = m_duration_frequency.begin();
  for (auto current = m_duration_frequency.begin(); m_duration_frequency.end() != current; ++current)
    if (current->second > most_often->second)
      most_often = current;

  // No duration at all!? No frame?
  if (m_duration_frequency.end() == most_often) {
    mxdebug_if(m_debug_timecodes, boost::format("Duration frequency: none found, using 25 FPS\n"));
    return 1000000000ll / 25;
  }

  auto best = std::make_pair(most_often->first, std::numeric_limits<uint64_t>::max());

  for (auto common_default_duration : s_common_default_durations) {
    uint64_t diff = std::abs(most_often->first - common_default_duration);
    if ((diff < 20000) && (diff < best.second))
      best = std::make_pair(common_default_duration, diff);
  }

  mxdebug_if(m_debug_timecodes, boost::format("Duration frequency. Result: %1%, diff %2%. Best before adjustment: %3%. All: %4%\n")
             % best.first % best.second % most_often->first
             % boost::accumulate(m_duration_frequency, std::string(""), [](std::string const &accu, std::pair<int64_t, int64_t> const &pair) {
                 return accu + (boost::format(" <%1% %2%>") % pair.first % pair.second).str();
               }));

  return best.first;
}

void
hevc::hevc_es_parser_c::cleanup() {
  if (m_frames.empty())
    return;

  if (m_discard_actual_frames) {
    m_stats.num_frames_discarded    += m_frames.size();
    m_stats.num_timecodes_discarded += m_provided_timecodes.size();

    m_frames.clear();
    m_provided_timecodes.clear();
    m_provided_stream_positions.clear();

    return;
  }

  auto frames_begin = m_frames.begin();
  auto frames_end   = m_frames.end();
  auto frame_itr    = frames_begin;

  // This may be wrong but is needed for mkvmerge to work correctly
  // (cluster_helper etc).
  frame_itr->m_keyframe = true;

  slice_info_t &idr         = frame_itr->m_si;
  sps_info_t &sps           = m_sps_info_list[idr.sps];
  bool simple_picture_order = false;

  if (   (   (HEVC_SLICE_TYPE_I   != idr.type)
          && (HEVC_SLICE_TYPE_SI  != idr.type)
          && (HEVC_SLICE_TYPE2_I  != idr.type)
          && (HEVC_SLICE_TYPE2_SI != idr.type))
      || (0 == idr.nal_ref_idc)
      || (0 != sps.pic_order_cnt_type)) {
    simple_picture_order = true;
    // return;
  }

  unsigned int idx                    = 0;
  unsigned int prev_pic_order_cnt_msb = 0;
  unsigned int prev_pic_order_cnt_lsb = 0;
  unsigned int pic_order_cnt_msb      = 0;

  while (frames_end != frame_itr) {
    slice_info_t &si = frame_itr->m_si;

    if (si.sps != idr.sps) {
      simple_picture_order = true;
      break;
    }

    if (frames_begin == frame_itr)
      ;

    else if ((si.pic_order_cnt_lsb < prev_pic_order_cnt_lsb) && ((prev_pic_order_cnt_lsb - si.pic_order_cnt_lsb) >= (1u << (sps.log2_max_pic_order_cnt_lsb - 1))))
      pic_order_cnt_msb = prev_pic_order_cnt_msb + (1 << sps.log2_max_pic_order_cnt_lsb);

    else if ((si.pic_order_cnt_lsb > prev_pic_order_cnt_lsb) && ((si.pic_order_cnt_lsb - prev_pic_order_cnt_lsb) > (1u << (sps.log2_max_pic_order_cnt_lsb - 1))))
      pic_order_cnt_msb = prev_pic_order_cnt_msb - (1 << sps.log2_max_pic_order_cnt_lsb);

    else
      pic_order_cnt_msb = prev_pic_order_cnt_msb;

    frame_itr->m_presentation_order = pic_order_cnt_msb + si.pic_order_cnt_lsb;
    frame_itr->m_decode_order       = idx;

    ++frame_itr;
    ++idx;

    if (0 != si.nal_ref_idc) {
      prev_pic_order_cnt_lsb = si.pic_order_cnt_lsb;
      prev_pic_order_cnt_msb = pic_order_cnt_msb;
    }
  }

  if (!simple_picture_order)
    brng::sort(m_frames, [](const hevc_frame_t &f1, const hevc_frame_t &f2) { return f1.m_presentation_order < f2.m_presentation_order; });

  brng::sort(m_provided_timecodes);

  frames_begin               = m_frames.begin();
  frames_end                 = m_frames.end();
  auto previous_frame_itr    = frames_begin;
  auto provided_timecode_itr = m_provided_timecodes.begin();
  for (frame_itr = frames_begin; frames_end != frame_itr; ++frame_itr) {
    if (frame_itr->m_has_provided_timecode) {
      frame_itr->m_start = *provided_timecode_itr;
      ++provided_timecode_itr;

      if (frames_begin != frame_itr)
        previous_frame_itr->m_end = frame_itr->m_start;

    } else {
      frame_itr->m_start = frames_begin == frame_itr ? m_max_timecode : previous_frame_itr->m_end;
      ++m_stats.num_timecodes_generated;
    }

    if (frame_itr->m_start == 1033333333)
      mxinfo("muh\n");
    frame_itr->m_end = frame_itr->m_start + duration_for(frame_itr->m_si);

    previous_frame_itr = frame_itr;
  }

  m_max_timecode = m_frames.back().m_end;
  m_provided_timecodes.erase(m_provided_timecodes.begin(), provided_timecode_itr);

  mxdebug_if(m_debug_timecodes, boost::format("CLEANUP frames <pres_ord dec_ord has_prov_tc tc dur>: %1%\n")
             % boost::accumulate(m_frames, std::string(""), [](std::string const &accu, hevc_frame_t const &frame) {
                 return accu + (boost::format(" <%1% %2% %3% %4% %5%>") % frame.m_presentation_order % frame.m_decode_order % frame.m_has_provided_timecode % frame.m_start % (frame.m_end - frame.m_start)).str();
               }));


  if (!simple_picture_order)
    brng::sort(m_frames, [](const hevc_frame_t &f1, const hevc_frame_t &f2) { return f1.m_decode_order < f2.m_decode_order; });

  frames_begin       = m_frames.begin();
  frames_end         = m_frames.end();
  previous_frame_itr = frames_begin;
  for (frame_itr = frames_begin; frames_end != frame_itr; ++frame_itr) {
    if (frames_begin != frame_itr)
      frame_itr->m_ref1 = previous_frame_itr->m_start - frame_itr->m_start;

    previous_frame_itr = frame_itr;
    m_duration_frequency[frame_itr->m_end - frame_itr->m_start]++;

    if (frame_itr->m_si.field_pic_flag)
      ++m_stats.num_field_slices;
    else
      ++m_stats.num_field_slices;
  }

  m_stats.num_frames_out += m_frames.size();
  m_frames_out.insert(m_frames_out.end(), frames_begin, frames_end);
  m_frames.clear();
}

memory_cptr
hevc::hevc_es_parser_c::create_nalu_with_size(const memory_cptr &src,
                                                   bool add_extra_data) {
  int final_size = m_nalu_size_length + src->get_size(), offset = 0, size;
  unsigned char *buffer;

  if (add_extra_data) {
    for (auto &mem : m_extra_data)
      final_size += mem->get_size();
    buffer = (unsigned char *)safemalloc(final_size);

    for (auto &mem : m_extra_data) {
      memcpy(buffer + offset, mem->get_buffer(), mem->get_size());
      offset += mem->get_size();
    }

    m_extra_data.clear();
  } else
    buffer = (unsigned char *)safemalloc(final_size);

  size = src->get_size();
  write_nalu_size(buffer + offset, size);
  memcpy(buffer + offset + m_nalu_size_length, src->get_buffer(), size);

  return memory_cptr(new memory_c(buffer, final_size, true));
}

memory_cptr
hevc::hevc_es_parser_c::get_hevcc()
  const {
  return hevcc_c{static_cast<unsigned int>(m_nalu_size_length), m_sps_list, m_pps_list}.pack();
}

bool
hevc::hevc_es_parser_c::has_par_been_found()
  const {
  assert(m_hevcc_ready);
  return m_par_found;
}

int64_rational_c const &
hevc::hevc_es_parser_c::get_par()
  const {
  assert(m_hevcc_ready && m_par_found);
  return m_par;
}

std::pair<int64_t, int64_t> const
hevc::hevc_es_parser_c::get_display_dimensions(int width,
                                                    int height)
  const {
  assert(m_hevcc_ready && m_par_found);

  if (0 >= width)
    width = get_width();
  if (0 >= height)
    height = get_height();

  return std::make_pair<int64_t, int64_t>(1 <= m_par ? irnd(width * boost::rational_cast<double>(m_par)) : width,
                                          1 <= m_par ? height                                            : irnd(height / boost::rational_cast<double>(m_par)));
}

size_t
hevc::hevc_es_parser_c::get_num_field_slices()
  const {
  return m_stats.num_field_slices;
}

size_t
hevc::hevc_es_parser_c::get_num_frame_slices()
  const {
  return m_stats.num_frame_slices;
}

void
hevc::hevc_es_parser_c::dump_info()
  const {
  mxinfo("Dumping m_frames_out:\n");
  for (auto &frame : m_frames_out) {
    mxinfo(boost::format("size %1% key %2% start %3% end %4% ref1 %5% adler32 0x%|6$08x|\n")
           % frame.m_data->get_size()
           % frame.m_keyframe
           % format_timecode(frame.m_start)
           % format_timecode(frame.m_end)
           % format_timecode(frame.m_ref1)
           % calc_adler32(frame.m_data->get_buffer(), frame.m_data->get_size()));
  }
}

std::string
hevc::hevc_es_parser_c::get_nalu_type_name(int type)
  const {
  auto name = m_nalu_names_by_type.find(type);
  return (m_nalu_names_by_type.end() == name) ? "unknown" : name->second;
}

void
hevc::hevc_es_parser_c::init_nalu_names() {
/*  m_nalu_names_by_type[HEVC_NALU_TYPE_IDR_W_RADL] = "non IDR slice";
  m_nalu_names_by_type[HEVC_NALU_TYPE_DP_A_SLICE]    = "DP A slice";
  m_nalu_names_by_type[HEVC_NALU_TYPE_DP_B_SLICE]    = "DP B slice";
  m_nalu_names_by_type[HEVC_NALU_TYPE_DP_C_SLICE]    = "DP C slice";*/
  m_nalu_names_by_type[HEVC_NALU_TYPE_IDR_W_RADL]     = "IDR slice";
  m_nalu_names_by_type[HEVC_NALU_TYPE_PREFIX_SEI]    = "PREFIX SEI";
  m_nalu_names_by_type[HEVC_NALU_TYPE_SUFFIX_SEI]    = "SUFFIX SEI";
  m_nalu_names_by_type[HEVC_NALU_TYPE_VIDEO_PARAM]   = "VIDEO param";
  m_nalu_names_by_type[HEVC_NALU_TYPE_SEQ_PARAM]     = "SEQ param";
  m_nalu_names_by_type[HEVC_NALU_TYPE_PIC_PARAM]     = "PIC param";
  m_nalu_names_by_type[HEVC_NALU_TYPE_ACCESS_UNIT]   = "access unit";
  m_nalu_names_by_type[HEVC_NALU_TYPE_END_OF_SEQ]    = "end of sequence";
  m_nalu_names_by_type[HEVC_NALU_TYPE_END_OF_STREAM] = "end of stream";
  m_nalu_names_by_type[HEVC_NALU_TYPE_FILLER_DATA]   = "filler";
}
