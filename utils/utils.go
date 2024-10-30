package utils

import (
	"fmt"
	"io/ioutil"
	// "os"
)

// ReadFile reads the contents of a file and returns it as a string.
// It returns an error if the file cannot be read.
func ReadFile(filename string) (string, error) {
	data, err := ioutil.ReadFile(filename)
	if err != nil {
		return "", fmt.Errorf("failed to read file %s: %w", filename, err)
	}
	return string(data), nil
}

// WriteFile writes the given content to a file.
// If the file does not exist, it will be created. If it does exist, it will be overwritten.
// It returns an error if the file cannot be written.
func WriteFile(filename, content string) error {
	err := ioutil.WriteFile(filename, []byte(content), 0644)
	if err != nil {
		return fmt.Errorf("failed to write file %s: %w", filename, err)
	}
	return nil
}
