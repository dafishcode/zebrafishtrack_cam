echo ".BATCH Convert to MP4 from Pgm. 2017 kostasl"
echo "Make HD Vids from Image Sequence Dirs.."

fps=$2
for dir in `find $1 -type d`
do
  #test -d "$dir" || continue
	files=($dir/*.pgm)
	if [ ${#files[@]} -gt 0 ]; then
#	 echo $dir;
	 echo "Found PGM Image FILES..in $dir";
#	 Make Video
	  filename=${dir//[\/]/_}
	  filename=${filename//[.]/}
	  echo $filename
	   avconv -framerate $fps -i $dir/%10d.pgm -c:v libx264 -crf 17 -crf_max 45 $dir/../../$filename.mp4
	fi


# Do something with $dir...
done

