/*=============================================================================
   Copyright (c) 2016 Cycfi Research

   Distributed under the MIT License [ https://opensource.org/licenses/MIT ]
=============================================================================*/
#if !defined(CYCFI_MIDI_HPP_OCTOBER_8_2012)
#define CYCFI_MIDI_HPP_OCTOBER_8_2012

#include <stdint.h>
#include <energia.h>

namespace cycfi { namespace midi
{
   namespace status
   {
      enum
      {
         note_off             = 0x80,
         note_on              = 0x90,
         poly_aftertouch      = 0xA0,
         control_change       = 0xB0,
         program_change       = 0xC0,
         channel_aftertouch   = 0xD0,
         pitch_bend           = 0xE0,
         sysex                = 0xF0,
         song_position        = 0xF2,
         song_select          = 0xF3,
         tune_request         = 0xF6,
         sysex_end            = 0xF7,
         timing_tick          = 0xF8,
         start                = 0xFA,
         continue_            = 0xFB,
         stop                 = 0xFC,
         active_sensing       = 0xFE,
         reset                = 0xFF
      };
   }

   namespace cc
   {
      enum controller
      {
         bank_select          = 0x00,
         modulation           = 0x01,
         breath               = 0x02,
         foot                 = 0x04,
         portamento_time      = 0x05,
         data_entry           = 0x06,
         channel_volume       = 0x07,
         balance              = 0x08,
         pan                  = 0x0A,
         expression           = 0x0B,
         effect_1             = 0x0C,
         effect_2             = 0x0D,
         general_1            = 0x10,
         general_2            = 0x11,
         general_3            = 0x12,
         general_4            = 0x13,

         bank_select_lsb      = 0x20,
         modulation_lsb       = 0x21,
         breath_lsb           = 0x22,
         foot_lsb             = 0x24,
         portamento_time_lsb  = 0x25,
         data_entry_lsb       = 0x26,
         channel_volume_lsb   = 0x27,
         balance_lsb          = 0x28,
         pan_lsb              = 0x2A,
         expression_lsb       = 0x2B,
         effect_1_lsb         = 0x2C,
         effect_2_lsb         = 0x2D,
         general_1_lsb        = 0x30,
         general_2_lsb        = 0x31,
         general_3_lsb        = 0x32,
         general_4_lsb        = 0x33,

         sustain              = 0x40,
         portamento           = 0x41,
         sostenuto            = 0x42,
         soft_pedal           = 0x43,
         legato               = 0x44,
         hold_2               = 0x45,

         sound_controller_1   = 0x46,  // default: sound variation
         sound_controller_2   = 0x47,  // default: timbre / harmonic content
         sound_controller_3   = 0x48,  // default: release time
         sound_controller_4   = 0x49,  // default: attack time
         sound_controller_5   = 0x4A,  // default: brightness
         sound_controller_6   = 0x4B,  // no default
         sound_controller_7   = 0x4C,  // no default
         sound_controller_8   = 0x4D,  // no default
         sound_controller_9   = 0x4E,  // no default
         sound_controller_10  = 0x4F,  // no default

         general_5            = 0x50,
         general_6            = 0x51,
         general_7            = 0x52,
         general_8            = 0x53,

         portamento_control   = 0x54,
         effects_1_depth      = 0x5B, // previously reverb send
         effects_2_depth      = 0x5C, // previously tremolo depth
         effects_3_depth      = 0x5D, // previously chorus depth
         effects_4_depth      = 0x5E, // previously celeste (detune) depth
         effects_5_depth      = 0x5F, // previously phaser effect depth
         data_inc             = 0x60, // increment data value (+1)
         data_dec             = 0x61, // decrement data value (-1)

         nonrpn_lsb           = 0x62,
         nonrpn_msb           = 0x63,
         rpn_lsb              = 0x64,
         rpn_msb              = 0x65,
         all_sounds_off       = 0x78,
         reset                = 0x79,
         local                = 0x7A,
         all_notes_off        = 0x7B,
         omni_off             = 0x7C,
         omni_on              = 0x7D,
         mono                 = 0x7E,
         poly                 = 0x7F
      };
   }

   ////////////////////////////////////////////////////////////////////////////
   // message: A generic MIDI message and I/O routines
   ////////////////////////////////////////////////////////////////////////////
   template <int size_>
   struct message
   {
      static int const size = size_;
      uint8_t data[size];
   };

   struct midi_stream
   {
      midi_stream()
      {
      }

      void start()
      {
         Serial.begin(31250);
      }

      midi_stream& operator<<(uint8_t val)
      {
         Serial.write(val);
         return *this;
      }
   };

   template <int size>
   inline midi_stream&
   operator<<(midi_stream& out, message<size> const& msg)
   {
      for (int i = 0; i < size; ++i)
         out << msg.data[i];
      return out;
   }

   ////////////////////////////////////////////////////////////////////////////
   // note_off
   ////////////////////////////////////////////////////////////////////////////
   struct note_off : message<3>
   {
      note_off(uint8_t channel, uint8_t key, uint8_t velocity)
      {
         data[0] = channel | status::note_off;
         data[1] = key;
         data[2] = velocity;
      }

      uint8_t channel() const { return data[0] & 0x0F; }
      uint8_t key() const { return data[1]; }
      uint8_t velocity() const { return data[2]; }
   };

   ////////////////////////////////////////////////////////////////////////////
   // note_on
   ////////////////////////////////////////////////////////////////////////////
   struct note_on : message<3>
   {
      note_on(uint8_t channel, uint8_t key, uint8_t velocity)
      {
         data[0] = channel | status::note_on;
         data[1] = key;
         data[2] = velocity;
      }

      uint8_t channel() const { return data[0] & 0x0F; }
      uint8_t key() const { return data[1]; }
      uint8_t velocity() const { return data[2]; }
   };

   ////////////////////////////////////////////////////////////////////////////
   // poly_aftertouch
   ////////////////////////////////////////////////////////////////////////////
   struct poly_aftertouch : message<3>
   {
      poly_aftertouch(uint8_t channel, uint8_t key, uint8_t pressure)
      {
         data[0] = channel | status::poly_aftertouch;
         data[1] = key;
         data[2] = pressure;
      }

      uint8_t channel() const { return data[0] & 0x0F; }
      uint8_t key() const { return data[1]; }
      uint8_t pressure() const { return data[2]; }
   };

   ////////////////////////////////////////////////////////////////////////////
   // control_change
   ////////////////////////////////////////////////////////////////////////////
   struct control_change : message<3>
   {
      typedef cc::controller controller_type;
      control_change(uint8_t channel, controller_type ctrl, uint8_t value)
      {
         data[0] = channel | status::control_change;
         data[1] = ctrl;
         data[2] = value;
      }

      uint8_t channel() const { return data[0] & 0x0F; }
      controller_type controller() const { return controller_type(data[1]); }
      uint8_t value() const { return data[2]; }
   };

   ////////////////////////////////////////////////////////////////////////////
   // program_change
   ////////////////////////////////////////////////////////////////////////////
   struct program_change : message<2>
   {
      program_change(uint8_t channel, uint8_t preset)
      {
         data[0] = channel | status::program_change;
         data[1] = preset;
      }

      uint8_t channel() const { return data[0] & 0x0F; }
      uint8_t preset() const { return data[1]; }
   };

   ////////////////////////////////////////////////////////////////////////////
   // channel_aftertouch
   ////////////////////////////////////////////////////////////////////////////
   struct channel_aftertouch : message<2>
   {
      channel_aftertouch(uint8_t channel, uint8_t pressure)
      {
         data[0] = channel | status::channel_aftertouch;
         data[1] = pressure;
      }

      uint8_t channel() const { return data[0] & 0x0F; }
      uint8_t pressure() const { return data[1]; }
   };

   ////////////////////////////////////////////////////////////////////////////
   // pitch_bend
   ////////////////////////////////////////////////////////////////////////////
   struct pitch_bend : message<3>
   {
      pitch_bend(uint8_t channel, uint16_t value)
      {
         data[0] = channel | status::pitch_bend;
         data[1] = value & 0x7F;
         data[2] = value >> 7;
      }

      pitch_bend(uint8_t channel, uint16_t lsb, uint8_t msb)
      {
         data[0] = channel | status::pitch_bend;
         data[1] = lsb;
         data[2] = msb;
      }

      uint8_t channel() const { return data[0] & 0x0F; }
      uint16_t value() const { return data[1] | (data[2] << 7); }
   };

   ////////////////////////////////////////////////////////////////////////////
   // song_position
   ////////////////////////////////////////////////////////////////////////////
   struct song_position : message<3>
   {
      song_position(uint16_t position)
      {
         data[0] = status::song_position;
         data[1] = position & 0x7F;
         data[2] = position >> 7;
      }

      song_position(uint8_t lsb, uint8_t msb)
      {
         data[0] = status::song_position;
         data[1] = lsb;
         data[2] = msb;
      }

      uint16_t position() const { return data[1] | (data[2] << 7); }
   };

   ////////////////////////////////////////////////////////////////////////////
   // song_select
   ////////////////////////////////////////////////////////////////////////////
   struct song_select : message<2>
   {
      song_select(uint8_t song_number)
      {
         data[0] = status::song_select;
         data[1] = song_number;
      }

      uint16_t song_number() const { return data[1]; }
   };

   ////////////////////////////////////////////////////////////////////////////
   // tune_request
   ////////////////////////////////////////////////////////////////////////////
   struct tune_request : message<1>
   {
      tune_request()
      {
         data[0] = status::tune_request;
      }
   };

   ////////////////////////////////////////////////////////////////////////////
   // timing_tick
   ////////////////////////////////////////////////////////////////////////////
   struct timing_tick : message<1>
   {
      timing_tick()
      {
         data[0] = status::timing_tick;
      }
   };

   ////////////////////////////////////////////////////////////////////////////
   // start
   ////////////////////////////////////////////////////////////////////////////
   struct start : message<1>
   {
      start()
      {
         data[0] = status::start;
      }
   };

   ////////////////////////////////////////////////////////////////////////////
   // continue_
   ////////////////////////////////////////////////////////////////////////////
   struct continue_ : message<1>
   {
      continue_()
      {
         data[0] = status::continue_;
      }
   };

   ////////////////////////////////////////////////////////////////////////////
   // stop
   ////////////////////////////////////////////////////////////////////////////
   struct stop : message<1>
   {
      stop()
      {
         data[0] = status::stop;
      }
   };

   ////////////////////////////////////////////////////////////////////////////
   // active_sensing
   ////////////////////////////////////////////////////////////////////////////
   struct active_sensing : message<1>
   {
      active_sensing()
      {
         data[0] = status::active_sensing;
      }
   };

   ////////////////////////////////////////////////////////////////////////////
   // reset
   ////////////////////////////////////////////////////////////////////////////
   struct reset : message<1>
   {
      reset()
      {
         data[0] = status::reset;
      }
   };

   ////////////////////////////////////////////////////////////////////////////
   // sysex
   ////////////////////////////////////////////////////////////////////////////
   template <int size_>
   struct sysex : message<size_ + 5>
   {
      sysex(uint16_t id, uint8_t const* data_in)
      {
         this->data[0] = status::sysex;
         this->data[1] = 0;
         this->data[2] = id >> 8;
         this->data[3] = id & 0x7f;
         this->data[size_ + 4] = status::sysex_end;
         for (int i = 0; i != size_; ++i)
            this->data[i+4] = data_in[i] & 0x7f;
      }

      uint16_t id() const { return (this->data[2] << 8) | this->data[3]; }
   };
}}

#endif
