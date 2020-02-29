if [ ! -f README.md ]; then
	echo "Wrong directory"    
	exit 0
fi

echo "Remaking..."
rm -rf xcode
mkdir xcode
cd xcode


cmake .. -G Xcode
