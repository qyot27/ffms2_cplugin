//  Copyright (c) 2010-2011 FFmpegSource Project
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.

#include "ffms.h"
#include "avs_common.h"
#include "ff_filters.h"

#ifdef MSVC
#include <assert.h>
#define _Static_assert static_assert // for clang-cl at least 12.0
#endif // MSVC

#define MAX_CACHE_FILE_LENGTH 512 // Windows API should explode before getting this long

static int default_cache_file( const char *src, const char *user_cache_file, char *out_cache_file )
{
    int ret = 0;
    if( !strcmp( user_cache_file, "" ) )
    {
        strcpy( out_cache_file, src );
        strcat( out_cache_file, ".ffindex" );
    }
    else
    {
        strcpy( out_cache_file, user_cache_file );
        ret = !_stricmp( src, user_cache_file );
    }
    return ret;
}

static AVS_Value AVSC_CC create_FFIndex( AVS_ScriptEnvironment *env, AVS_Value args, void *user_data )
{
    enum { Source, Cachefile, Indexmask, Dumpmask, Audiofile, Errorhandling, Overwrite, Utf8, Enable_drefs, Use_absolute_path };

    FFMS_Init( 0, as_bool( as_elt( args, Utf8), 0 ) );
    init_ErrorInfo( ei );

    AVS_Value elt0 = as_elt( args, Source);
    if( !avs_is_string( elt0 ) )
        return avs_new_value_error( "FFIndex: No source specified" );

    const char *src = as_string( elt0, NULL );
    const char *user_cache_file = as_string( as_elt( args, Cachefile), "" );
    int index_mask = as_int( as_elt( args, Indexmask), -1 );
    int dump_mask = as_int( as_elt( args, Dumpmask), 0 );
    const char *audio_file = as_string( as_elt( args, Audiofile), "%sourcefile%.%trackzn%.w64" );
    int err_handler = as_int( as_elt( args, Errorhandling), FFMS_IEH_IGNORE );
    char overwrite = as_bool( as_elt( args, Overwrite), 0 );
    char enable_drefs = as_bool(as_elt(args, Enable_drefs), 0);
    char use_absolute_path = as_bool(as_elt(args, Use_absolute_path), 0);

    char cache_file[MAX_CACHE_FILE_LENGTH];
    if( default_cache_file( src, user_cache_file, cache_file ) )
        return avs_new_value_error( "FFIndex: Cache will overwrite the source" );

    if( !strcmp( audio_file, "" ) )
        return avs_new_value_error( "FFIndex: Specifying an empty audio filename is not allowed" );

    FFMS_Index *index = FFMS_ReadIndex( cache_file, &ei );
    if( overwrite || !index || (index && FFMS_IndexBelongsToFile( index, src, 0 ) != FFMS_ERROR_SUCCESS) )
    {
        FFMS_KeyValuePair LAVFOpts[] = { { "enable_drefs", enable_drefs ? "1" : "0" }, { "use_absolute_path", use_absolute_path ? "1" : "0" } };
        FFMS_Indexer *indexer = FFMS_CreateIndexer2( src, LAVFOpts, 2, &ei );
        if( !indexer )
            return avs_new_value_error( ffms_avs_lib.avs_sprintf( env, "FFIndex: %s", ei.Buffer ) );

        /* Treat -1 as meaning track numbers above sizeof(int) too, dumping implies indexing */
        if (dump_mask == -1) {
            FFMS_TrackTypeIndexSettings( indexer, FFMS_TYPE_AUDIO, 1, 1 );
        } else if (index_mask == -1) {
            FFMS_TrackTypeIndexSettings( indexer, FFMS_TYPE_AUDIO, 1, 0 );
        }

        /* Apply attributes to remaining tracks (will set the attributes again on some tracks) */
        for (int i = 0; i < sizeof(index_mask) * 8; i++) {
            int temp = (((index_mask >> i) & 1) | ((dump_mask >> i) & 1));
            if (temp)
                FFMS_TrackIndexSettings( indexer, i, temp, (dump_mask >> i) & 1 );
        }

        index = FFMS_DoIndexing2( indexer, err_handler, &ei );
        if( !index )
            return avs_new_value_error( ffms_avs_lib.avs_sprintf( env, "FFIndex: %s", ei.Buffer ) );
        if( FFMS_WriteIndex( cache_file, index, &ei ) )
        {
            FFMS_DestroyIndex( index );
            return avs_new_value_error( ffms_avs_lib.avs_sprintf( env, "FFIndex: %s", ei.Buffer ) );
        }
        FFMS_DestroyIndex( index );
        return avs_new_value_int( 2 - overwrite );
    }
    FFMS_DestroyIndex( index );
    return avs_new_value_int( 0 );
}

static AVS_Value AVSC_CC create_FFVideoSource( AVS_ScriptEnvironment *env, AVS_Value args, void *user_data )
{
    enum { Source, Track, Cache, Cachefile, Fpsnum, Fpsden, Threads, Timecodes, Seekmode, Rffmode, Width, Height, Resizer, Colorspace, Utf8, Varprefix };

    FFMS_Init( 0, as_bool( as_elt( args, Utf8), 0 ) );
    init_ErrorInfo( ei );

    AVS_Value elt0 = as_elt( args, 0 );
    if( !avs_is_string( elt0 ) )
        return avs_new_value_error( "FFVideoSource: No source specified" );

    const char *src = as_string(elt0, NULL );
    int track = as_int( as_elt( args, Track), -1 );
    char cache = as_bool( as_elt( args, Cache), 1 );
    const char *user_cache_file = as_string( as_elt( args, Cachefile), "" );
    int fps_num = as_int( as_elt( args, Fpsnum), -1 );
    int fps_den = as_int( as_elt( args, Fpsden), 1 );
    int threads = as_int( as_elt( args, Threads), -1 );
    const char *timecodes = as_string( as_elt( args, Timecodes), "" );
    int seek_mode = as_int( as_elt( args, Seekmode), 1 );
    int rff_mode = as_int( as_elt( args, Rffmode), 0 );
    int width = as_int( as_elt( args, Width), 0 );
    int height = as_int( as_elt( args, Height), 0 );
    const char *resizer = as_string( as_elt( args, Resizer), "BICUBIC" );
    const char *csp_name = as_string( as_elt( args, Colorspace), "" );
    const char *var_prefix = as_string( as_elt( args, Varprefix), "" );

    if( fps_den < 1 )
        return avs_new_value_error( "FFVideoSource: FPS denominator needs to be 1 or higher" );
    if( track <= -2 )
        return avs_new_value_error( "FFVideoSource: No video track selected" );
    if( seek_mode < -1 || seek_mode > 3 )
        return avs_new_value_error( "FFVideoSource: Invalid seekmode selected" );
    if( rff_mode < 0 || rff_mode > 2 )
        return avs_new_value_error( "FFVideoSource: Invalid RFF mode selected" );
    if( rff_mode > 0 && fps_num > 0 )
        return avs_new_value_error( "FFVideoSource: RFF modes may not be combined with CFR conversion" );
    if( !_stricmp( src, timecodes ) )
        return avs_new_value_error( "FFVideoSource: Timecodes will overwrite the source" );

    char cache_file[MAX_CACHE_FILE_LENGTH];
    if( default_cache_file( src, user_cache_file, cache_file ) )
        return avs_new_value_error( "FFVideoSource: Cache will overwrite the source" );

    FFMS_Index *index = NULL;
    if( cache )
    {
        index = FFMS_ReadIndex( cache_file, &ei );
        if( index && *user_cache_file && FFMS_IndexBelongsToFile( index, src, 0 ) != FFMS_ERROR_SUCCESS )
        {
            FFMS_DestroyIndex( index );
            index = NULL;
        }
    }
    if( !index )
    {
        FFMS_Indexer *indexer = FFMS_CreateIndexer( src, &ei );
        if( !indexer )
            return avs_new_value_error( ffms_avs_lib.avs_sprintf( env, "FFVideoSource: %s", ei.Buffer ) );

        index = FFMS_DoIndexing2( indexer, FFMS_IEH_CLEAR_TRACK, &ei );
        if( !index )
            return avs_new_value_error( ffms_avs_lib.avs_sprintf( env, "FFVideoSource: %s", ei.Buffer ) );

        if( cache )
            if( FFMS_WriteIndex( cache_file, index, &ei ) )
            {
                FFMS_DestroyIndex( index );
                return avs_new_value_error( ffms_avs_lib.avs_sprintf( env, "FFVideoSource: %s", ei.Buffer ) );
            }
    }

    if( track == -1 )
        track = FFMS_GetFirstIndexedTrackOfType( index, FFMS_TYPE_VIDEO, &ei );
    if( track < 0 )
        return avs_new_value_error( "FFVideoSource: No video track found" );

    if( strcmp( timecodes, "" ) )
        if( FFMS_WriteTimecodes( FFMS_GetTrackFromIndex( index, track ), timecodes, &ei ) )
        {
            FFMS_DestroyIndex( index );
            return avs_new_value_error( ffms_avs_lib.avs_sprintf( env, "FFVideoSource: %s", ei.Buffer ) );
        }

    AVS_Value video = FFVideoSource_create( env, src, track, index, fps_num, fps_den, threads,
        seek_mode, rff_mode, width, height, resizer, csp_name, var_prefix );
    if( avs_is_error( video ) )
    {
        FFMS_DestroyIndex( index );
        index = NULL;
    }
    return video;
}

static AVS_Value AVSC_CC create_FFAudioSource( AVS_ScriptEnvironment *env, AVS_Value args, void *user_data )
{
    enum{ Source, Track, Cache, Cachefile, Adjustdelay, Utf8, Fill_gaps, Drc_scale, Varprefix };

    FFMS_Init( 0, as_bool( as_elt( args, Utf8), 0 ) );
    init_ErrorInfo( ei );

    if( !avs_is_string( as_elt( args, Source) ) )
        return avs_new_value_error( "FFAudioSource: No source specified" );

    const char *src = as_string( as_elt( args, Source), NULL );
    int track = as_int( as_elt( args, Track), -1 );
    char cache = as_bool( as_elt( args, Cache), 1 );
    const char *user_cache_file = as_string( as_elt( args, Cachefile), "" );
    int adjust_delay = as_int( as_elt( args, Adjustdelay), -1 );
    int fill_gaps = as_int(as_elt(args, Fill_gaps), -1);
    double drc_scale = as_float(as_elt(args, Drc_scale), 0.0f);
    const char *var_prefix = as_string( as_elt( args, Varprefix), "" );

    if( track <= -2 )
        return avs_new_value_error( "FFAudioSource: No audio track selected" );

    char cache_file[MAX_CACHE_FILE_LENGTH];
    if( default_cache_file( src, user_cache_file, cache_file ) )
        return avs_new_value_error( "FFAudioSource: Cache will overwrite the source" );

    FFMS_Index *index = NULL;
    if( cache )
    {
        index = FFMS_ReadIndex( cache_file, &ei );
        if( index && *user_cache_file && FFMS_IndexBelongsToFile( index, src, 0 ) != FFMS_ERROR_SUCCESS )
        {
            FFMS_DestroyIndex( index );
            index = NULL;
        }
    }

    // Index needs to be remade if it is an unindexed audio track
    if( index && track >= 0 && track < FFMS_GetNumTracks( index ) &&
        FFMS_GetTrackType( FFMS_GetTrackFromIndex( index, track ) ) == FFMS_TYPE_AUDIO &&
        !FFMS_GetNumFrames( FFMS_GetTrackFromIndex( index, track ) ) )
    {
        FFMS_DestroyIndex( index );
        index = NULL;
    }

    // More complicated for finding a default track, reindex the file if at least one audio track exists
    if( index && FFMS_GetFirstTrackOfType( index, FFMS_TYPE_AUDIO, &ei ) >= 0 &&
        FFMS_GetFirstIndexedTrackOfType( index, FFMS_TYPE_AUDIO, &ei ) < 0 )
    {
        int i;
        for( i = 0; i < FFMS_GetNumTracks( index ); i++ )
            if( FFMS_GetTrackType( FFMS_GetTrackFromIndex( index, i ) ) == FFMS_TYPE_AUDIO )
            {
                FFMS_DestroyIndex( index );
                index = NULL;
                break;
            }
    }

    if( !index )
    {
        FFMS_Indexer *indexer = FFMS_CreateIndexer( src, &ei );
        if( !indexer )
            return avs_new_value_error( ffms_avs_lib.avs_sprintf( env, "FFAudioSource: %s", ei.Buffer ) );

        FFMS_TrackTypeIndexSettings( indexer, FFMS_TYPE_AUDIO, 1, 0);

        index = FFMS_DoIndexing2( indexer, FFMS_IEH_CLEAR_TRACK, &ei );
        if( !indexer )
            return avs_new_value_error( ffms_avs_lib.avs_sprintf( env, "FFAudioSource: %s", ei.Buffer ) );

        if( cache )
            if( FFMS_WriteIndex( cache_file, index, &ei ) )
            {
                FFMS_DestroyIndex( index );
                return avs_new_value_error( ffms_avs_lib.avs_sprintf( env, "FFAudioSource: %s", ei.Buffer ) );
            }
    }

    if( track == -1 )
        track = FFMS_GetFirstIndexedTrackOfType( index, FFMS_TYPE_AUDIO, &ei );
    if( track < 0 )
        return avs_new_value_error( "FFAudioSource: No audio track found" );

    if( adjust_delay < -3 )
        return avs_new_value_error( "FFAudioSource: Invalid delay adjustment specified" );
    if( adjust_delay >= FFMS_GetNumTracks( index ) )
        return avs_new_value_error( "FFAudioSource: Invalid track to calculate delay from specified" );


    AVS_Value audio = FFAudioSource_create( env, src, track, index, adjust_delay, fill_gaps, drc_scale, var_prefix );
    if( avs_is_error( audio ) )
    {
        FFMS_DestroyIndex( index );
        index = NULL;
    }
    return audio;
}

static AVS_Value AVSC_CC create_FFmpegSource2(AVS_ScriptEnvironment* env, AVS_Value args, void* user_data)
{
    enum { Source, Vtrack, Atrack, Cache, Cachefile, Fpsnum, Fpsden, Threads, Timecodes, Seekmode, Overwrite, Width, Height, Resizer, Colorspace, Rffmode, Adjustdelay, Utf8, Enable_drefs, Use_absolute_path, Fill_gaps, Drc_scale, Varprefix };

    const char* FFIArgNames[] = { "source", "cachefile", "indexmask", "overwrite", "utf8", "enable_drefs", "use_absolute_path" };
    const char* FFVArgNames[] = { "source", "track", "cache", "cachefile", "fpsnum", "fpsden", "threads", "timecodes", "seekmode", "rffmode", "width", "height", "resizer", "colorspace", "utf8", "varprefix" };
    const char* FFAArgNames[] = { "source", "track", "cache", "cachefile", "adjustdelay", "utf8", "fill_gaps", "drc_scale", "varprefix" };

    char cache = as_bool(as_elt(args, Cache), 1);
    char WithAudio = as_int(as_elt(args, Atrack), -2) > -2;
    AVS_Value ffindex = avs_void;
    if (cache)
    {
        AVS_Value FFIArgs[] = { as_elt(args, Source), as_elt(args, Cachefile), WithAudio ? avs_new_value_int(-1) : avs_new_value_int(0), as_elt(args, Overwrite), as_elt(args, Utf8), as_elt(args, Enable_drefs), as_elt(args, Use_absolute_path) };
        _Static_assert((sizeof(FFIArgs) / sizeof(FFIArgs[0])) == (sizeof(FFIArgNames) / sizeof(FFIArgNames[0])), "FFIndex: arg error");
        ffindex = ffms_avs_lib.avs_invoke(env, "FFIndex", avs_new_value_array(FFIArgs, 7), FFIArgNames);
        if (avs_is_error(ffindex))
        {
            ffms_avs_lib.avs_release_value(ffindex);
            return avs_new_value_error("FFIndex: invoke error");
        }
    }

    AVS_Value FFVArgs[] = { as_elt(args, Source), as_elt(args, Vtrack), as_elt(args, Cache), as_elt(args, Cachefile), as_elt(args, Fpsnum), as_elt(args, Fpsden), as_elt(args, Threads), as_elt(args, Timecodes), as_elt(args, Seekmode),
        as_elt(args, Rffmode), as_elt(args, Width), as_elt(args, Height), as_elt(args, Resizer), as_elt(args, Colorspace), as_elt(args, Utf8), as_elt(args, Varprefix) };
    if ((sizeof(FFVArgs) / sizeof(FFVArgs[0])) != (sizeof(FFVArgNames) / sizeof(FFVArgNames[0])))
    {
        ffms_avs_lib.avs_release_value(ffindex);
        return avs_new_value_error("FFVideoSource: arg error");
    }

    AVS_Value Video = ffms_avs_lib.avs_invoke(env, "FFVideoSource", avs_new_value_array(FFVArgs, 16), FFVArgNames);
    if (avs_is_error(Video))
    {
        ffms_avs_lib.avs_release_value(ffindex);
        ffms_avs_lib.avs_release_value(Video);
        return avs_new_value_error("FFVideoSource: invoke error");
    }

    if (WithAudio)
    {
        AVS_Value FFAArgs[] = { as_elt(args, Source), as_elt(args, Atrack), as_elt(args, Cache), as_elt(args, Cachefile), as_elt(args, Adjustdelay), as_elt(args, Utf8), as_elt(args, Fill_gaps), as_elt(args, Drc_scale), as_elt(args, Varprefix) };
        if ((sizeof(FFAArgs) / sizeof(FFAArgs[0])) != (sizeof(FFAArgNames) / sizeof(FFAArgNames[0])))
        {
            ffms_avs_lib.avs_release_value(ffindex);
            ffms_avs_lib.avs_release_value(Video);
            return avs_new_value_error("FFAudioSource: arg error");
        }

        AVS_Value Audio = ffms_avs_lib.avs_invoke(env, "FFAudioSource", avs_new_value_array(FFAArgs, 9), FFAArgNames);
        if (avs_is_error(Audio))
        {
            ffms_avs_lib.avs_release_value(ffindex);
            ffms_avs_lib.avs_release_value(Video);
            ffms_avs_lib.avs_release_value(Audio);
            return avs_new_value_error("FFVideoSource: invoke error");
        }

        AVS_Value ADArgs[] = { Video, Audio };
        AVS_Value audio_dub = ffms_avs_lib.avs_invoke(env, "AudioDubEx", avs_new_value_array(ADArgs, 2), 0);
        ffms_avs_lib.avs_release_value(ffindex);
        ffms_avs_lib.avs_release_value(Video);
        ffms_avs_lib.avs_release_value(Audio);

        if (avs_is_error(audio_dub))
        {
            ffms_avs_lib.avs_release_value(audio_dub);
            return avs_new_value_error("AudioDubEx: invoke error");
        }

        return audio_dub;
    }
    else
    {
        ffms_avs_lib.avs_release_value(ffindex);
        return Video;
    }
}

static AVS_Value create_FFImageSource(AVS_ScriptEnvironment* env, AVS_Value args, void* user_data)
{
    enum { Source, Width, Height, Resizer, Colorspace, Utf8, Varprefix };

    const char* FFISArgNames[] = { "source", "width", "height", "resizer", "colorspace", "utf8", "varprefix", "cache", "seekmode" };
    AVS_Value FFISArgs[] = { as_elt(args, Source), as_elt(args, Width), as_elt(args, Height), as_elt(args, Resizer), as_elt(args, Colorspace), as_elt(args, Utf8), as_elt(args, Varprefix), avs_new_value_bool(false), avs_new_value_int(-1) };
    _Static_assert((sizeof(FFISArgs) / sizeof(FFISArgs[0])) == (sizeof(FFISArgNames) / sizeof(FFISArgNames[0])), "FFImageSource: arg error");
    return ffms_avs_lib.avs_invoke(env, "FFVideoSource", avs_new_value_array(FFISArgs, 9), FFISArgNames);
}

static AVS_Value create_FFCopyrightInfringement(AVS_ScriptEnvironment* env, AVS_Value args, void* user_data)
{
    const char* ArgNames[] = { "source" };
    AVS_Value ffindex = ffms_avs_lib.avs_invoke(env, "FFIndex", args, ArgNames);
    if (avs_is_error(ffindex))
    {
        ffms_avs_lib.avs_release_value(ffindex);
        return avs_new_value_error("FFIndex: invoke error");
    }

    AVS_Value ffvideo = ffms_avs_lib.avs_invoke(env, "FFVideoSource", args, ArgNames);
    if (avs_is_error(ffvideo))
    {
        ffms_avs_lib.avs_release_value(ffindex);
        ffms_avs_lib.avs_release_value(ffvideo);
        return avs_new_value_error("FFVideoSource: invoke error");
    }

    AVS_Value ffaudio = ffms_avs_lib.avs_invoke(env, "FFAudioSource", args, ArgNames);
    if (avs_is_error(ffaudio))
    {
        ffms_avs_lib.avs_release_value(ffindex);
        ffms_avs_lib.avs_release_value(ffvideo);
        ffms_avs_lib.avs_release_value(ffaudio);
        return avs_new_value_error("FFAudioSource: invoke error");
    }

    AVS_Value ExArgs[2] = { ffvideo, ffaudio };
    AVS_Value output = ffms_avs_lib.avs_invoke(env, "AudioDubEx", avs_new_value_array(ExArgs, 2), 0);
    ffms_avs_lib.avs_release_value(ffindex);
    ffms_avs_lib.avs_release_value(ffvideo);
    ffms_avs_lib.avs_release_value(ffaudio);

    return output;
}

static AVS_Value AVSC_CC create_FFGetLogLevel( AVS_ScriptEnvironment *env, AVS_Value args, void *user_data )
{ return avs_new_value_int( FFMS_GetLogLevel() ); }

static AVS_Value AVSC_CC create_FFSetLogLevel( AVS_ScriptEnvironment *env, AVS_Value args, void *user_data )
{
    FFMS_SetLogLevel( as_int( as_elt( args, 0 ), 0 ) );
    return avs_new_value_int( FFMS_GetLogLevel() );
}

static AVS_Value AVSC_CC create_FFGetVersion( AVS_ScriptEnvironment *env, AVS_Value args, void *user_data )
{
    int vint = FFMS_GetVersion();
    return avs_new_value_string( ffms_avs_lib.avs_sprintf(env, "%d.%d.%d.%d", vint >> 24, (vint >> 16) & 0xFF, (vint >> 8) & 0xFF, vint & 0xFF) );
}

/* the AVS loader for LoadCPlugin */
const char *AVSC_CC avisynth_c_plugin_init( AVS_ScriptEnvironment* env )
{
    /* load the avs library */
    if( ffms_load_avs_lib( env ) )
        return "Failure";
    ffms_avs_lib.avs_add_function( env, "FFIndex", "[source]s[cachefile]s[indexmask]i[dumpmask]i[audiofile]s[errorhandling]i[overwrite]b[utf8]b[enable_drefs]b[use_absolute_path]b", create_FFIndex, 0 );
    ffms_avs_lib.avs_add_function( env, "FFVideoSource", "[source]s[track]i[cache]b[cachefile]s[fpsnum]i[fpsden]i[threads]i[timecodes]s[seekmode]i[rffmode]i[width]i[height]i[resizer]s[colorspace]s[utf8]b[varprefix]s", create_FFVideoSource, 0 );
    ffms_avs_lib.avs_add_function( env, "FFAudioSource", "[source]s[track]i[cache]b[cachefile]s[adjustdelay]i[utf8]b[fill_gaps]i[drc_scale]f[varprefix]s", create_FFAudioSource, 0 );
    ffms_avs_lib.avs_add_function( env, "FFmpegSource2", "[source]s[vtrack]i[atrack]i[cache]b[cachefile]s[fpsnum]i[fpsden]i[threads]i[timecodes]s[seekmode]i[overwrite]b[width]i[height]i[resizer]s[colorspace]s[rffmode]i[adjustdelay]i[utf8]b[enable_drefs]b[use_absolute_path]b[fill_gaps]i[drc_scale]f[varprefix]s", create_FFmpegSource2, 0 );
    ffms_avs_lib.avs_add_function( env, "FFMS2", "[source]s[vtrack]i[atrack]i[cache]b[cachefile]s[fpsnum]i[fpsden]i[threads]i[timecodes]s[seekmode]i[overwrite]b[width]i[height]i[resizer]s[colorspace]s[rffmode]i[adjustdelay]i[utf8]b[enable_drefs]b[use_absolute_path]b[fill_gaps]i[drc_scale]f[varprefix]s", create_FFmpegSource2, 0 );
    ffms_avs_lib.avs_add_function( env, "FFImageSource", "[source]s[width]i[height]i[resizer]s[colorspace]s[utf8]b[varprefix]s", create_FFImageSource, 0 );
    ffms_avs_lib.avs_add_function( env, "FFCopyrightInfringement", "[source]s", create_FFCopyrightInfringement, 0 );
    ffms_avs_lib.avs_add_function( env, "FFGetLogLevel", "", create_FFGetLogLevel, 0 );
    ffms_avs_lib.avs_add_function( env, "FFSetLogLevel", "i", create_FFSetLogLevel, 0 );
    ffms_avs_lib.avs_add_function( env, "FFGetVersion", "", create_FFGetVersion, 0 );

    /* tell avs to call our cleanup method when it closes */
    ffms_avs_lib.avs_at_exit( env, ffms_free_avs_lib, 0 );
    return "FFmpegSource - The Second Coming V2.0 Final";
}
