echo ".BATCH Convert to Low Res MP4 from Pgm. 2017 kostasl"
echo "Checks for Encoding Error during file encoding"
echo "Make HD Vids from Image Sequence Dirs.."

fps=$2
outdir=$3
for dir in `find $1 -type d`
do
  #test -d "$dir" || continue
	files=($dir/*.pgm)
	if [ ${#files[@]} -gt 0 ]; then
#	 echo $dir;
	 echo "Found PGM Image FILES..in $dir \n";
#	 Make Video
	  filename=${dir//[\/]/_}
	  filename=${filename//[.]/}
	  echo $filename
	   encerror="$(avconv -v warning -framerate $fps -i $dir/%10d.pgm -c:v libx264 -pix_fmt yuv420p -crf 17 -crf_max 33 $outdir/$filename.mp4)"
	  echo "$encerror"
          echo "$encerror" > $outdir/avconvError.log 
#           avconv -v error -i $outdir/$filename.mp4 -map 0:0 -f null - 2>error.log
	fi


##Check Vid Files 
find . -name "$outdir/*.mp4" -exec avconv -v error -i "{}" -map 0:1 -f null - 2>error.log \;
##For Separate log Files :
#find . -name "*.mp4" -exec sh -c "ffmpeg -v error -i {} -map 0:1 -f null - 2>{}.log" \;
# Do something with $dir...
done

